#include "hidjoystickrptparser.h"

// ---------------------------------------------------------
// Parse関数: USBから届いた生データを構造体に流し込む
// ---------------------------------------------------------
void JoystickReportParser::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
    if (!joyEvents) return;

    // Uno R3のフリーズを避けるため、Parse内は最小限のコピー処理のみを実行
    joyEvents->packetLen = len;
    for (uint8_t i = 0; i < len && i < 12; i++) {
        joyEvents->rawBuf[i] = buf[i];
    }

    joyEvents->isNative = true;
    joyEvents->OnGamePadChanged((const GamePadEventData*)buf);
}

// ---------------------------------------------------------
// OnGamePadChanged関数: 解明されたアライメントに基づいてデータを格納
// ---------------------------------------------------------
void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
    // 14bit (55 ~ 16318) ステアリング抽出
    steer = ((uint16_t)(evt->SteeringHi & 0x3F) << 8) | (uint16_t)evt->SteeringLo;
    
    // 反転ペダル (非踏下255 ~ 踏下0) を 0 ~ 255 に正常補正
    accel = 255 - evt->Accel;
    brake = 255 - evt->Brake;
    
    buttons1 = evt->buttons1;
    buttons2 = evt->buttons2;
    buttons3 = evt->buttons3;
    dpad     = evt->dpad;
}