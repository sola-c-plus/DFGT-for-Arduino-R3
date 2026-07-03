#if !defined(__HIDJOYSTICKRPTPARSER_H__)
#define __HIDJOYSTICKRPTPARSER_H__

#include <usbhid.h>

// DFGT ネイティブモードのデータ構造体（完全解明版）
struct GamePadEventData {
    uint8_t buttons1;    // 0: B0 ◯✗△□ + D-Pad
    uint8_t buttons2;    // 1: B1 L1/R1/L2/R2/L3/R3/SELECT/START
    uint8_t buttons3;    // 2: B2 シフト/赤ダイヤル系/ハンドル＋−/Centerボタン
    uint8_t dpad;        // 3: B3 PSボタン
    uint8_t SteeringLo;  // 4: B4 ハンドル下位
    uint8_t SteeringHi;  // 5: B5 ハンドル上位
    uint8_t Accel;       // 6: B6 アクセル (反転仕様)
    uint8_t Brake;       // 7: B7 ブレーキ (反転仕様)
    uint8_t dummy8;      // 8
    uint8_t dummy9;      // 9
    uint8_t dummy10;     // 10
    uint8_t dummy11;     // 11
} __attribute__((packed));

// データを保持するクラス
class JoystickEvents {
public:
    uint16_t steer;
    uint8_t accel;
    uint8_t brake;
    uint8_t buttons1;
    uint8_t buttons2;
    uint8_t buttons3;
    uint8_t dpad;
    bool isNative;
    
    uint8_t packetLen;
    uint8_t rawBuf[12];

    JoystickEvents() : steer(0), accel(0), brake(0), buttons1(0), buttons2(0), buttons3(0), dpad(0), isNative(false), packetLen(0) {
        memset(rawBuf, 0, 12);
    }
    virtual void OnGamePadChanged(const GamePadEventData *evt);
};

// HIDレポート解析クラス
class JoystickReportParser : public HIDReportParser {
    JoystickEvents *joyEvents;

public:
    JoystickReportParser(JoystickEvents *evt) : joyEvents(evt) {}
    virtual void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
};

#endif // __HIDJOYSTICKRPTPARSER_H__