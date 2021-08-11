import argparse
import os
from typing import Dict, List, Tuple

import fontTools.ttLib
import fontTools.pens.recordingPen
import jinja2

HEADER_TEMPLATE = """
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
"""

SOURCE_TEMPLATE = """
#include "subway_ticker/font.hh"

namespace app::font::subway_ticker {
namespace {
const Font font = {
  .name = "subway_ticker",
  .glyph_from_codepoint = {
  {% for codepoint, name in name_by_codepoint.items() %}
      {{- codepoint -}, Glyph{.name = "{- name -}", .data = {{- format_data(data_from_glyph[name]) -}}}},
  {% endfor %}
  }
};
}

const Font &get_font() {
  return font;
}
}
"""

def format_data(data: List[int]) -> str:
    return ', '.join([str(x) for x in data])

def font_coordinate_to_idx(x: int, y: int):
    x_coords = [62, 187, 312, 437, 562, 687, 813]
    y_coords = [99, 224, 349, 474, 599, 724, 849]
    x_is_match = lambda val: abs(x - val) <= 1
    y_is_match = lambda val: abs(y - val) <= 1
    x_matches = list(filter(x_is_match, x_coords))
    y_matches = list(filter(y_is_match, y_coords))
    assert len(x_matches), f'unable to find match for x: {x}'
    assert len(y_matches), f'unable to find match for y: {y}'
    return x_coords.index(x_matches[0]), y_coords.index(y_matches[0])

def width_px_to_width_blocks(w: int):
    width_options = [0, 223, 348, 473, 598, 723, 848, 973]
    is_match = lambda val: abs(w - val) <= 20
    w_match = list(filter(is_match, width_options))
    assert len(w_match), f'unable to find match for w: {w}'
    return width_options.index(w_match[0])

def get_extrema(pts: List[Tuple[int, int]]):
    min_x = float('inf')
    max_y = -float('inf')

    for pt in pts:
        if pt[0] < min_x:
            min_x = pt[0]
        if pt[1] > max_y:
            max_y = pt[1]

    return min_x, max_y

def split_into_blocks(recording):
    blocks = []
    pts = []
    for pt in recording:
        if pt[0] in ['moveTo', 'lineTo']:
            pts.append(pt[1][0])
        elif pt[0] == 'closePath':
            if len(pts) >= 4:
                x, y = get_extrema(pts)
                blocks.append(font_coordinate_to_idx(x, y))
            pts = []
    return blocks

def get_blocks_from_font(font: fontTools.ttLib.TTFont) -> Dict[str, List[Tuple[int, int]]]:
    glyph_set = font.getGlyphSet()

    blocks_from_glyph = {}

    for glyph_name in glyph_set.keys():
        glyph = glyph_set[glyph_name]
        recording_pen = fontTools.pens.recordingPen.RecordingPen()
        glyph.draw(recording_pen)
        try:
            blocks = split_into_blocks(recording_pen.value)
        except Exception as e:
            continue
        blocks_from_glyph[glyph_name] = blocks

    return blocks_from_glyph

def get_data_from_glyphs(font: fontTools.ttLib.TTFont, blocks_from_glyph: Dict[str, List[Tuple[int, int]]]):
    glyph_set = font.getGlyphSet()
    data_from_glyph = {}
    for glyph_name, blocks in blocks_from_glyph.items():
        glyph = glyph_set[glyph_name]
        try:
            width = width_px_to_width_blocks(glyph.width)
        except:
            print(f'skipping glyph: {glyph_name} w: {glyph.width}')
            continue
        data = [0] * width
        for block in blocks:
            assert block[1] >= 0 and block[1] <= 6, f'{glyph_name} is weird height: {block}'
            data[block[0]] = data[block[0]] | ( 1 << (6-block[1]))
        data_from_glyph[glyph_name] = data
    return data_from_glyph


def get_cpp_from_data(font: fontTools.ttLib.TTFont, data_from_glyph: Dict[str, List[int]]):
    t = jinja2.Template(SOURCE_TEMPLATE, variable_start_string='{-', variable_end_string='-}',
                        trim_blocks=True, lstrip_blocks=True)
    name_by_codepoint = {codepoint: name for codepoint, name in font.getBestCmap().items() if codepoint < 128}

    cpp = (t.render(name_by_codepoint=name_by_codepoint, data_from_glyph=data_from_glyph, format_data=format_data))
    hh = HEADER_TEMPLATE

    return hh, cpp


def convert_to_cpp(input_path: str, output_path: str):
    font = fontTools.ttLib.TTFont(input_path)
    blocks_from_glyph = get_blocks_from_font(font)
    data_from_glyph = get_data_from_glyphs(font, blocks_from_glyph)
    hh, cpp = get_cpp_from_data(font, data_from_glyph)

    with open(os.path.join(output_path, 'font.hh'), 'w') as file_out:
        file_out.write(hh)

    with open(os.path.join(output_path, 'font.cc'), 'w') as file_out:
        file_out.write(cpp)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Convert TTF to C++ source files');
    parser.add_argument('--input', help='input file name', required=True)
    parser.add_argument('--output', help='output file name', required=True)
    args = parser.parse_args()

    convert_to_cpp(args.input, args.output)
