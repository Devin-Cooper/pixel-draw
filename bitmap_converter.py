from PIL import Image
import tkinter as tk
from tkinter import filedialog
import math

# Unit conversion functions
def mm_to_inches(mm):
   return mm / 25.4

def inches_to_mm(inches):
   return inches * 25.4

# Calculate pixel size to fit within specified document dimensions
def get_pixel_size_from_doc_size(doc_width_mm, doc_height_mm, pixel_width, pixel_height):
   width_pixel_size = doc_width_mm / pixel_width
   height_pixel_size = doc_height_mm / pixel_height
   return min(width_pixel_size, height_pixel_size)  # Use smallest size to fit within bounds

# Calculate final document dimensions based on pixel size
def get_output_dimensions(width, height, pixel_size_mm):
   width_mm = width * pixel_size_mm
   height_mm = height * pixel_size_mm
   return width_mm, height_mm

# Generate zigzag pattern points for a pixel square
def create_zigzag_path(x, y, width, height, stroke_width, angle):
   angle_rad = math.radians(angle)
   spacing = stroke_width  # Space lines by stroke width to avoid overlap
   
   # Calculate number of lines needed to fill space
   diagonal = math.sqrt(width**2 + height**2)
   num_lines = int(diagonal / spacing) + 1
   
   # Calculate perpendicular vector for offset
   perp_vector = (-math.sin(angle_rad), math.cos(angle_rad))
   
   path = []
   for i in range(num_lines):
       offset = i * spacing
       start_x = x + offset * perp_vector[0]
       start_y = y + offset * perp_vector[1]
       
       # Calculate end point at angle
       end_x = start_x + width * math.cos(angle_rad)
       end_y = start_y + width * math.sin(angle_rad)
       
       # Clip line to pixel bounds
       points = clip_line_to_rect(start_x, start_y, end_x, end_y, x, y, width, height)
       if points:
           path.append(points)
   
   return path

# Line clipping helper functions
def clip_line_to_rect(x1, y1, x2, y2, rx, ry, rw, rh):
   if not line_rect_intersect(x1, y1, x2, y2, rx, ry, rw, rh):
       return None
       
   if x1 < rx: x1, y1 = intersect_vertical(x1, y1, x2, y2, rx)
   if x1 > rx + rw: x1, y1 = intersect_vertical(x1, y1, x2, y2, rx + rw)
   if y1 < ry: x1, y1 = intersect_horizontal(x1, y1, x2, y2, ry)
   if y1 > ry + rh: x1, y1 = intersect_horizontal(x1, y1, x2, y2, ry + rh)
   
   if x2 < rx: x2, y2 = intersect_vertical(x1, y1, x2, y2, rx)
   if x2 > rx + rw: x2, y2 = intersect_vertical(x1, y1, x2, y2, rx + rw)
   if y2 < ry: x2, y2 = intersect_horizontal(x1, y1, x2, y2, ry)
   if y2 > ry + rh: x2, y2 = intersect_horizontal(x1, y1, x2, y2, ry + rh)
   
   return (x1, y1, x2, y2)

def line_rect_intersect(x1, y1, x2, y2, rx, ry, rw, rh):
   return not (max(x1, x2) < rx or min(x1, x2) > rx + rw or 
              max(y1, y2) < ry or min(y1, y2) > ry + rh)

def intersect_vertical(x1, y1, x2, y2, x):
   t = (x - x1) / (x2 - x1)
   y = y1 + t * (y2 - y1)
   return x, y

def intersect_horizontal(x1, y1, x2, y2, y):
   t = (y - y1) / (y2 - y1)
   x = x1 + t * (x2 - x1)
   return x, y

# Main SVG generation function
def bitmap_to_svg(input_file, output_file, pixel_size_mm, stroke_width_mm, angle=None):
   # Open and convert image to 1-bit bitmap
   img = Image.open(input_file).convert('1')
   width, height = img.size
   
   # Convert measurements from mm to points (72dpi)
   pixel_size = (pixel_size_mm / 25.4) * 72
   stroke_width = (stroke_width_mm / 25.4) * 72
   inset = stroke_width / 2  # Inset boxes by half stroke width
   
   with open(output_file, 'w') as f:
       # Write SVG header
       f.write(f'<svg width="{width*pixel_size}px" height="{height*pixel_size}px" '
               f'viewBox="0 0 {width*pixel_size} {height*pixel_size}" '
               'xmlns="http://www.w3.org/2000/svg">\n')
       
       # Process each pixel
       for y in range(height):
           for x in range(width):
               if img.getpixel((x,y)) == 0:  # Black pixel
                   px = x * pixel_size + inset
                   py = y * pixel_size + inset
                   pwidth = pixel_size - stroke_width
                   pheight = pixel_size - stroke_width
                   
                   # Always draw the pixel outline
                   f.write(f'  <rect x="{px}" y="{py}" '
                          f'width="{pwidth}" height="{pheight}" '
                          f'fill="none" stroke="black" stroke-width="{stroke_width}"/>\n')
                   
                   # Add zigzag pattern if specified
                   if angle is not None:
                       paths = create_zigzag_path(px, py, pwidth, pheight, stroke_width, angle)
                       for p in paths:
                           f.write(f'  <line x1="{p[0]}" y1="{p[1]}" x2="{p[2]}" y2="{p[3]}" '
                                  f'stroke="black" stroke-width="{stroke_width}" />\n')
       f.write('</svg>')

# Main program
def main():
   root = tk.Tk()
   root.withdraw()  # Hide the tk window
   input_file = filedialog.askopenfilename(filetypes=[("Image files", "*.bmp *.gif")])

   if input_file:
       img = Image.open(input_file)
       width, height = img.size
       print(f"\nInput image dimensions: {width}x{height} pixels")
       
       while True:
           # Get size input method
           size_type = input("\nEnter (p) for pixel size or (d) for document size: ").lower()
           if size_type not in ['p', 'd']:
               print("Please enter either 'p' or 'd'")
               continue

           # Get measurement units
           units = input("\nEnter units (mm/in): ").lower()
           if units not in ['mm', 'in']:
               print("Please enter either 'mm' or 'in'")
               continue
               
           # Calculate pixel size based on input method
           if size_type == 'p':
               size = float(input(f"\nEnter pixel size ({units}): "))
               pixel_size_mm = size if units == 'mm' else inches_to_mm(size)
           else:
               width_size = float(input(f"\nEnter document width ({units}): "))
               height_size = float(input(f"Enter document height ({units}): "))
               doc_width_mm = width_size if units == 'mm' else inches_to_mm(width_size)
               doc_height_mm = height_size if units == 'mm' else inches_to_mm(height_size)
               pixel_size_mm = get_pixel_size_from_doc_size(doc_width_mm, doc_height_mm, width, height)

           # Get stroke width
           stroke_size = float(input(f"\nEnter stroke width ({units}): "))
           stroke_width_mm = stroke_size if units == 'mm' else inches_to_mm(stroke_size)
           
           # Display final dimensions
           width_mm, height_mm = get_output_dimensions(width, height, pixel_size_mm)
           print(f"\nOutput document dimensions:")
           print(f"Width: {width_mm:.1f}mm ({mm_to_inches(width_mm):.1f}in)")
           print(f"Height: {height_mm:.1f}mm ({mm_to_inches(height_mm):.1f}in)")
           print(f"Pixel size: {pixel_size_mm:.2f}mm ({mm_to_inches(pixel_size_mm):.3f}in)")
           print(f"Stroke width: {stroke_width_mm:.2f}mm ({mm_to_inches(stroke_width_mm):.3f}in)")
           
           # Generate SVG if dimensions are accepted
           if input("\nContinue with these dimensions? (y/n): ").lower() == 'y':
               fill_type = input("\nEnter fill type (s for solid stroke, z for zigzag): ").lower()
               angle = None
               if fill_type == 'z':
                   angle = float(input("Enter zigzag angle (degrees): "))
               
               output_file = input_file.rsplit('.', 1)[0] + '.svg'
               bitmap_to_svg(input_file, output_file, pixel_size_mm, stroke_width_mm, angle)
               break

if __name__ == "__main__":
   main()