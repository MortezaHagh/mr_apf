import os
import cairosvg

def convert_svg_to_png(input_file, output_file):
    cairosvg.svg2png(url=input_file, write_to=output_file)

def convert_directory_svg_to_png(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.lower().endswith('.svg'):
                svg_file = os.path.join(root, file)
                png_file = os.path.splitext(svg_file)[0] + '.png'
                convert_svg_to_png(svg_file, png_file)
                print(f"Converted {svg_file} to {png_file}")

# Provide the directory path where the SVG files are located
directory_path = '/home/morteza/Desktop/No Obstacles/'

convert_directory_svg_to_png(directory_path)