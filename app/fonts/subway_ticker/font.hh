
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace app::font {
struct Glyph {
  std::string name;
  std::vector<std::uint8_t> data;
};

struct Font {
  std::string name;
  std::unordered_map<int, Glyph> glyph_from_codepoint;
};

namespace subway_ticker {
const Font &get_font();
}
}
