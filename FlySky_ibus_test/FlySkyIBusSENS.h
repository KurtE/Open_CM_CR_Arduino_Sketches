#if !defined(FlySkyIBusSENS_h)
#define FlySkyIBusSENS_h

#define IBUS_BAUDRATE 115200

enum {FLYSENS_DISCOVER, FLYSENS_TYPE, FLYSENS_VALUE};
typedef uint16_t (*FlySensCB)(uint8_t cb_type, uint8_t sens_index);

class FlySkyIBusSENS {
  public:
    FlySkyIBusSENS(UARTClass &hws, FlySensCB  callback)
      : serial_(hws)
      , packetPtr_(0)
      , serialInTime_(0)
      , callback_(callback)
    {
    };

    void begin()
    {
      // Assume Serial 1
      Serial.println("FlySkyIBusSENS::begin called");
      serial_.setDxlMode(true);
      drv_dxl_begin(0);
      serial_.begin(IBUS_BAUDRATE);
      drv_dxl_tx_enable(0, false);  // put in read mode.
    }

    void step()
    {
      if (!serial_.available()) return;   // nothing to do.
      while (serial_.available()) {
        int i = serial_.read();
        //Serial.print(".");
        if (((millis() - serialInTime_) > 3) || (packetPtr_ >= sizeof(packet_))) packetPtr_ = 0;  // Assume any delay is we are not aligned
            packet_[packetPtr_++] = i;  // save away the character.
        if (packetPtr_ == packet_[0]) {
          if (serial_.available()) digitalWrite(BOARD_LED_PIN, !digitalRead(BOARD_LED_PIN));
          // in theory we have a complete packet.
          // Does checksum match?
          uint8_t packet_size = packet_[0];
            uint16_t checksum_in = ((uint16_t)packet_[packet_size - 1] << 8) + packet_[packet_size - 2];
            uint16_t checksum_calc = 0xffff;
            for (uint8_t ib = 0; ib < (packet_size - 2); ib++) checksum_calc -= packet_[ib];
            if (checksum_in == checksum_calc) {
              // have a packet.  What type is it?
              if ((packet_[1] & 0xf0) == 0x80) {
                // Discovery packet
                if ((*callback_)(FLYSENS_DISCOVER, packet_[1] & 0x0f)) {
                  // It says we want to handle this device.
                  drv_dxl_tx_enable(0, true);  // put in write mode.
                  serial_.write(packet_, packet_size);  // echo the packet back
                  serial_.flush();  // wait until it outputs
                  drv_dxl_tx_enable(0, false);  // put back in Read mode
                }

              } else if ((packet_[1] & 0xf0) == 0x90) {
                // What type of packet do we have?
                uint8_t sensor_type = (*callback_)(FLYSENS_TYPE, packet_[1] & 0x0f);
                // It says we want to handle this device.
                drv_dxl_tx_enable(0, true);  // put in write mode.
                // Lets generate a return packet.
                checksum_calc = 0xffff - (6 + 2);
                serial_.write(6);             // packet size
                serial_.write(packet_[1]);    // packet type and sensor number
                checksum_calc -= packet_[1];
                serial_.write(sensor_type);  // sensor type from call back
                checksum_calc -= sensor_type;
                serial_.write(2);             // always 2? - maybe data size?
                serial_.write(checksum_calc & 0xff); // low byte checksum
                serial_.write(checksum_calc >> 8);  // high byte checksum
                serial_.flush();  // wait until it outputs
                drv_dxl_tx_enable(0, false);  // put back in Read mode
              } else if ((packet_[1] & 0xf0) == 0xA0) {
                // Return data.
                uint16_t sensor_data = (*callback_)(FLYSENS_VALUE, packet_[1] & 0x0f);
                // It says we want to handle this device.
                drv_dxl_tx_enable(0, true);  // put in write mode.
                // Lets generate a return packet.
                checksum_calc = 0xffff - 6;
                serial_.write(6);             // packet size
                serial_.write(packet_[1]);    // packet type and sensor number
                checksum_calc -= packet_[1];
                serial_.write(sensor_data & 0xff);  // sensor type from call back
                checksum_calc -= (sensor_data & 0xff);
                serial_.write(sensor_data >> 8);       // always 2? - maybe data size?
                checksum_calc -= (sensor_data >> 8);
                serial_.write(checksum_calc & 0xff); // low byte checksum
                serial_.write(checksum_calc >> 8);  // high byte checksum
                serial_.flush();  // wait until it outputs
                drv_dxl_tx_enable(0, false);  // put back in Read mode
              }
            } else {
              Serial.println("Checksum error");
            }
            packetPtr_ = 0;
          }
      }
      serialInTime_ = millis();
    }

  private:

    UARTClass &serial_;
    uint8_t       packetPtr_;
    uint8_t       packet_[10];
    uint32_t      serialInTime_;
    FlySensCB     callback_;
};

#endif
