#pragma once

// High level interface for SSD1306
struct SSD1306Config {};

template <typename Transport> class SSD1306 {
public:
  SSD1306(Transport transport) : transport_(transport) {}

  

private:
  Transport transport_;
}

  class SSD1306TransportBase {
  }
