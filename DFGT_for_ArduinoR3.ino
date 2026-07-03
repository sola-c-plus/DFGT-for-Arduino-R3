#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SPI.h>
#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

// ====================================================================
// 【ユーザー設定変数：FFB合成エンジン】
// すべての力をArduinoが物理演算し、調和させてDFGTへ送信します。
// 0 ~ 100 の間で自由に変更して、好みのフィーリングに調整してください。
// ==========================================
uint8_t springStrength = 100; // センターに戻るバネの強さ (0:するする ~ 100:最強)
                                   // ※ゲインを5倍強化したため、100にすると非常に強力に戻ります。
uint8_t damperStrength = 100; // ハンドルの粘り重さ・抵抗感 (0:無負荷 ~ 100:極めて重い)
// ====================================================================

// タイムライン制御用の変数
unsigned long attachTime = 0;
bool switchCommandSent = false;
bool configCommandSent = false;

// リアルタイム・FFB合成用の変数
int16_t lastSteer = 8192;
uint8_t lastSentForce = 128; // 128 が力ゼロ(中立)の基準値

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 2000; // シリアルプリントの間隔: 2秒に1回

// 移行コマンド (Output Report, ep=0, iface=0, type=2, id=0, len=8)
void requestNativeMode() {
    Serial.println(F("[SYSTEM] Sending Native Switch Commands..."));
    uint8_t revert[] = {0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Hid.SetReport(0, 0, 2, 0, 8, revert);
    delay(250);

    uint8_t native[] = {0xf8, 0x09, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00};
    Hid.SetReport(0, 0, 2, 0, 8, native);
}

// 900度レンジを設定する関数 (Output Report, ep=0, iface=0, type=2, id=0, len=8)
void configureNativeWheel() {
    uint8_t range[] = {0xf8, 0x81, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00};
    Hid.SetReport(0, 0, 2, 0, 8, range);
}

// ====================================================================
// 【Arduino自律制御：FFB（バネ ＋ ダンパー）合成エンジン】
// ====================================================================
void updateFFBEngine() {
    int16_t currentSteer = JoyEvents.steer;

    // 1. センターからの偏位量（バネ用）の計算
    int16_t displacement = currentSteer - 8192; // 8192 が中立

    // 2. ステアリングの回転速度（ダンパー用）を計算
    int16_t diff = currentSteer - lastSteer;
    lastSteer = currentSteer;

    // 微小な測定ノイズをカット（静止時の微細なガタつきを防止）
    if (abs(diff) < 3) {
        diff = 0;
    }

    // --- 各種エフェクトの物理演算 ---
    int16_t spring_offset = 0;
    int16_t damper_offset = 0;

    // 【スプリング（復元力）の計算：ゲインを大幅に強化】
    if (springStrength > 0) {
        // センターからのズレ(displacement)が3000（約160度）に達した時点で
        // 設定した強さの最大値になるようにスケーリング (約5倍のトルクに強化)
        float spring_k = (float)springStrength * 70.0 / 3000.0;
        spring_offset = (int16_t)(displacement * spring_k);

        // バネ単体の最大力リミッターを 70 まで大幅拡張
        int16_t limit = map(springStrength, 0, 100, 0, 70);
        spring_offset = constrain(spring_offset, -limit, limit);
    }

    // 【ダンパー（減衰・抵抗力）の計算】
    if (damperStrength > 0 && diff != 0) {
        // 粘り重さのゲインも、よりダイレクトに体感できるように調整
        float damper_k = (float)damperStrength * 50.0 / 250.0; 
        damper_offset = (int16_t)(diff * damper_k);

        // ダンパー単体の最大力リミッターを 50 まで拡張
        int16_t limit = map(damperStrength, 0, 100, 0, 50);
        damper_offset = constrain(damper_offset, -limit, limit);
    }

    // --- 【重要：極性の修正】 ---
    // バネは「中央に戻す（加算）」、ダンパーは「回転を妨げる（加算）」正しい物理極性に修正します。
    // calculated = 128 + spring_offset + damper_offset
    int16_t calculated = 128 + spring_offset + damper_offset;
    
    // モーター保護のため 8 ~ 248 の安全範囲でクリップ (モーターのほぼ最大トルクまで解放)
    uint8_t targetForce = constrain(calculated, 8, 248); 

    // 力の変化があったとき、または完全に静止して脱力（128）する瞬間のみモーターへ命令を送信
    if (targetForce != lastSentForce) {
        uint8_t constant_force[] = {0x11, 0x00, targetForce, 0x00, 0x00, 0x00, 0x00, 0x00};
        Hid.SetReport(0, 0, 2, 0, 8, constant_force);
        lastSentForce = targetForce;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000); // 同期確立待ち

    if (Usb.Init() == -1) {
        Serial.println(F("OSC did not start."));
    }
    delay(200);
    Hid.SetReportParser(0, &Joy);
    
    Serial.println(F("System Ready. Please connect DFGT..."));
}

void loop() {
    Usb.Task();

    if (Hid.isReady()) {
        uint8_t addr = Hid.GetAddress();

        // 接続直後に基準タイマーをスタート
        if (attachTime == 0) {
            attachTime = millis();
            switchCommandSent = false;
            configCommandSent = false;
            Serial.println(F("[SYSTEM] Wheel connected. Timer started."));
        }

        unsigned long elapsed = millis() - attachTime;

        // 【シーケンス1】接続から 1秒後：移行コマンドを送信
        if (elapsed >= 1000 && !switchCommandSent) {
            requestNativeMode();
            switchCommandSent = true;
        }

        // 【シーケンス2】接続から 4.5秒後：キャリブレーション回転（5秒間）をほぼ待って
        // レンジ設定とバネ・ダンパーの初期化を送信
        if (elapsed >= 4500 && !configCommandSent) {
            Serial.println(F("[SYSTEM] Setting 900deg & FFB synthesis initialization..."));
            configureNativeWheel(); 
            delay(50);
            
            // FFB合成エンジンがコンスタントフォースで上書きするため、ハードバネは完全オフ（f5）に設定
            uint8_t autocenter_off[] = {0xf5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            Hid.SetReport(0, 0, 2, 0, 8, autocenter_off);
            configCommandSent = true;
            
            // 速度計測の基準値をリセット
            lastSteer = JoyEvents.steer;
            Serial.println(F("[SYSTEM] Setup complete. FFB Synthesis Engine active!"));
        }

        // 【FFB合成エンジンのリアルタイム稼働】
        if (configCommandSent) {
            updateFFBEngine();
        }

        // 表示の更新タイミング制御 (2秒ごと)
        unsigned long currentTime = millis();
        if (currentTime - lastSendTime >= SEND_INTERVAL) {
            lastSendTime = currentTime;

            if (configCommandSent) {
                // 各ボタンのデコード
                uint8_t hat = JoyEvents.buttons1 & 0x0F;
                bool dpadUp    = (hat == 0 || hat == 1 || hat == 7) ? 1 : 0;
                bool dpadRight = (hat == 1 || hat == 2 || hat == 3) ? 1 : 0;
                bool dpadDown  = (hat == 3 || hat == 4 || hat == 5) ? 1 : 0;
                bool dpadLeft  = (hat == 5 || hat == 6 || hat == 7) ? 1 : 0;

                bool btnCross    = (JoyEvents.buttons1 & 0x10) ? 1 : 0; // ✗
                bool btnSquare   = (JoyEvents.buttons1 & 0x20) ? 1 : 0; // □
                bool btnCircle   = (JoyEvents.buttons1 & 0x40) ? 1 : 0; // ◯
                bool btnTriangle = (JoyEvents.buttons1 & 0x80) ? 1 : 0; // △

                bool btnPaddleR  = (JoyEvents.buttons2 & 0x01) ? 1 : 0; // パドル右 (R1)
                bool btnPaddleL  = (JoyEvents.buttons2 & 0x02) ? 1 : 0; // パドル左 (L1)
                bool btnR2       = (JoyEvents.buttons2 & 0x04) ? 1 : 0;
                bool btnL2       = (JoyEvents.buttons2 & 0x08) ? 1 : 0;
                bool btnSelect   = (JoyEvents.buttons2 & 0x10) ? 1 : 0;
                bool btnStart    = (JoyEvents.buttons2 & 0x20) ? 1 : 0;
                bool btnR3       = (JoyEvents.buttons2 & 0x40) ? 1 : 0;
                bool btnL3       = (JoyEvents.buttons2 & 0x80) ? 1 : 0;

                bool btnShiftPlus = (JoyEvents.buttons3 & 0x01) ? 1 : 0; // シフト＋
                bool btnShiftMin  = (JoyEvents.buttons3 & 0x02) ? 1 : 0; // シフトー
                bool btnDialEnt   = (JoyEvents.buttons3 & 0x04) ? 1 : 0; // 決定
                bool btnWheelPlus = (JoyEvents.buttons3 & 0x08) ? 1 : 0; // ハンドル＋
                bool btnDialPlus  = (JoyEvents.buttons3 & 0x10) ? 1 : 0; // ダイヤル右回転
                bool btnDialMin   = (JoyEvents.buttons3 & 0x20) ? 1 : 0; // ダイヤル左回転
                bool btnWheelMin  = (JoyEvents.buttons3 & 0x40) ? 1 : 0; // ハンドルー
                bool btnCenter    = (JoyEvents.buttons3 & 0x80) ? 1 : 0; // Centerボタン（ホーン）

                bool btnPS       = (JoyEvents.dpad & 0x01) ? 1 : 0; // PSボタン

                // --- シリアル出力 ---
                Serial.print(F("STEER:")); Serial.print(JoyEvents.steer);
                Serial.print(F(" ACC:"));  Serial.print(JoyEvents.accel);
                Serial.print(F(" BRK:"));  Serial.print(JoyEvents.brake);
                
                Serial.print(F(" [DPAD_U:")); Serial.print(dpadUp);
                Serial.print(F(" D:"));       Serial.print(dpadDown);
                Serial.print(F(" L:"));       Serial.print(dpadLeft);
                Serial.print(F(" R:"));       Serial.print(dpadRight);

                Serial.print(F("] [CRS:"));   Serial.print(btnCross);
                Serial.print(F(" SQR:"));     Serial.print(btnSquare);
                Serial.print(F(" CIR:"));     Serial.print(btnCircle);
                Serial.print(F(" TRI:"));     Serial.print(btnTriangle);

                Serial.print(F("] [PAD_L:")); Serial.print(btnPaddleL);
                Serial.print(F(" PAD_R:"));   Serial.print(btnPaddleR);
                Serial.print(F(" L2:"));      Serial.print(btnL2);
                Serial.print(F(" R2:"));      Serial.print(btnR2);
                Serial.print(F(" L3:"));      Serial.print(btnL3);
                Serial.print(F(" R3:"));      Serial.print(btnR3);

                Serial.print(F("] [SEL:"));   Serial.print(btnSelect);
                Serial.print(F(" STR:"));     Serial.print(btnStart);
                Serial.print(F(" PS:"));      Serial.print(btnPS);

                Serial.print(F("] [S_UP:"));  Serial.print(btnShiftPlus);
                Serial.print(F(" S_DN:"));    Serial.print(btnShiftMin);
                Serial.print(F(" H_UP:"));    Serial.print(btnWheelPlus);
                Serial.print(F(" H_DN:"));    Serial.print(btnWheelMin);

                Serial.print(F("] [ENT:"));   Serial.print(btnDialEnt);
                Serial.print(F(" D_+:"));     Serial.print(btnDialPlus);
                Serial.print(F(" -:"));        Serial.print(btnDialMin);
                Serial.print(F(" CNT:"));     Serial.print(btnCenter);
                Serial.println(F("]"));
                
            } else {
                Serial.print(F("[INFO] Initializing: elapsed "));
                Serial.print(elapsed / 1000.0);
                Serial.println(F("s / Please wait..."));
            }
        }
    } else {
        if (attachTime != 0) {
            attachTime = 0; 
            Serial.println(F("[SYSTEM] Device disconnected. Waiting for connection..."));
        }
    }
}