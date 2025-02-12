# Import required libraries
from PIL import Image
import tkinter as tk
from tkinter import ttk, filedialog
import math

# Unit conversion functions
def mm_to_inches(mm):
   return mm / 25.4

def inches_to_mm(inches):
   return inches * 25.4

# Calculate pixel size that fits within document dimensions
def get_pixel_size_from_doc_size(doc_width_mm, doc_height_mm, pixel_width, pixel_height):
   width_pixel_size = doc_width_mm / pixel_width
   height_pixel_size = doc_height_mm / pixel_height
   return min(width_pixel_size, height_pixel_size)

# Get final output dimensions in mm
def get_output_dimensions(width, height, pixel_size_mm):
   width_mm = width * pixel_size_mm
   height_mm = height * pixel_size_mm
   return width_mm, height_mm

# Generate zigzag pattern within pixel boundaries
def create_zigzag_path(x, y, width, height, stroke_width, angle):
   angle_rad = math.radians(angle)
   spacing = stroke_width
   diagonal = math.sqrt(width**2 + height**2)
   num_lines = int(diagonal / spacing) + 1
   perp_vector = (-math.sin(angle_rad), math.cos(angle_rad))
   
   path = []
   for i in range(num_lines):
       offset = i * spacing
       start_x = x + offset * perp_vector[0]
       start_y = y + offset * perp_vector[1]
       end_x = start_x + width * math.cos(angle_rad)
       end_y = start_y + width * math.sin(angle_rad)
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
   img = Image.open(input_file).convert('1')
   width, height = img.size
   pixel_size = (pixel_size_mm / 25.4) * 72
   stroke_width = (stroke_width_mm / 25.4) * 72
   inset = stroke_width / 2
   
   with open(output_file, 'w') as f:
       # Write SVG header
       f.write(f'<svg width="{width*pixel_size}px" height="{height*pixel_size}px" '
               f'viewBox="0 0 {width*pixel_size} {height*pixel_size}" '
               'xmlns="http://www.w3.org/2000/svg">\n')
       
       # Process each pixel
       for y in range(height):
           for x in range(width):
               if img.getpixel((x,y)) == 0:
                   px = x * pixel_size + inset
                   py = y * pixel_size + inset
                   pwidth = pixel_size - stroke_width
                   pheight = pixel_size - stroke_width
                   
                   # Draw pixel outline
                   f.write(f'  <rect x="{px}" y="{py}" '
                          f'width="{pwidth}" height="{pheight}" '
                          f'fill="none" stroke="black" stroke-width="{stroke_width}"/>\n')
                   
                   # Add zigzag if specified
                   if angle is not None:
                       paths = create_zigzag_path(px, py, pwidth, pheight, stroke_width, angle)
                       for p in paths:
                           f.write(f'  <line x1="{p[0]}" y1="{p[1]}" x2="{p[2]}" y2="{p[3]}" '
                                  f'stroke="black" stroke-width="{stroke_width}" />\n')
       f.write('</svg>')

# GUI Class
class BitmapConverterGUI:
   def __init__(self):
       self.root = tk.Tk()
       self.root.title("Bitmap to SVG Converter")
       self.setup_gui()
   
   # Setup GUI components    
   def setup_gui(self):
       # File selection frame
       file_frame = ttk.LabelFrame(self.root, text="Input File", padding="5 5 5 5")
       file_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
       
       self.file_path = tk.StringVar()
       ttk.Label(file_frame, textvariable=self.file_path).grid(row=0, column=0, padx=5)
       ttk.Button(file_frame, text="Browse", command=self.choose_file).grid(row=0, column=1)
       
       # Size settings frame
       size_frame = ttk.LabelFrame(self.root, text="Size Settings", padding="5 5 5 5")
       size_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
       
       # Size type radio buttons
       self.size_type = tk.StringVar(value="pixel")
       ttk.Radiobutton(size_frame, text="Pixel Size", variable=self.size_type, 
                      value="pixel", command=self.toggle_size_inputs).grid(row=0, column=0)
       ttk.Radiobutton(size_frame, text="Document Size", variable=self.size_type,
                      value="document", command=self.toggle_size_inputs).grid(row=0, column=1)
       
       # Units radio buttons
       self.units = tk.StringVar(value="mm")
       ttk.Radiobutton(size_frame, text="mm", variable=self.units,
                      value="mm").grid(row=1, column=0)
       ttk.Radiobutton(size_frame, text="inches", variable=self.units,
                      value="in").grid(row=1, column=1)
       
       # Pixel size input frame
       self.pixel_size_frame = ttk.Frame(size_frame)
       self.pixel_size_frame.grid(row=2, column=0, columnspan=2)
       ttk.Label(self.pixel_size_frame, text="Pixel Size:").grid(row=0, column=0)
       self.pixel_size = ttk.Entry(self.pixel_size_frame, width=10)
       self.pixel_size.grid(row=0, column=1)
       
       # Document size input frame
       self.doc_size_frame = ttk.Frame(size_frame)
       ttk.Label(self.doc_size_frame, text="Width:").grid(row=0, column=0)
       self.doc_width = ttk.Entry(self.doc_size_frame, width=10)
       self.doc_width.grid(row=0, column=1)
       ttk.Label(self.doc_size_frame, text="Height:").grid(row=1, column=0)
       self.doc_height = ttk.Entry(self.doc_size_frame, width=10)
       self.doc_height.grid(row=1, column=1)
       
       # Stroke settings frame
       stroke_frame = ttk.LabelFrame(self.root, text="Stroke Settings", padding="5 5 5 5")
       stroke_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
       
       ttk.Label(stroke_frame, text="Stroke Width:").grid(row=0, column=0)
       self.stroke_width = ttk.Entry(stroke_frame, width=10)
       self.stroke_width.grid(row=0, column=1)
       
       # Fill type radio buttons
       self.fill_type = tk.StringVar(value="solid")
       ttk.Radiobutton(stroke_frame, text="Solid", variable=self.fill_type,
                      value="solid", command=self.toggle_angle).grid(row=1, column=0)
       ttk.Radiobutton(stroke_frame, text="Zigzag", variable=self.fill_type,
                      value="zigzag", command=self.toggle_angle).grid(row=1, column=1)
       
       # Angle input frame
       self.angle_frame = ttk.Frame(stroke_frame)
       ttk.Label(self.angle_frame, text="Angle (degrees):").grid(row=0, column=0)
       self.angle = ttk.Entry(self.angle_frame, width=10)
       self.angle.grid(row=0, column=1)
       
       # Convert button
       ttk.Button(self.root, text="Convert", command=self.convert).grid(row=3, column=0, pady=10)
       
       self.toggle_size_inputs()
       self.toggle_angle()
   
   # File chooser dialog
   def choose_file(self):
       filename = filedialog.askopenfilename(filetypes=[("Image files", "*.bmp *.gif")])
       if filename:
           self.file_path.set(filename)
   
   # Toggle between pixel and document size inputs        
   def toggle_size_inputs(self):
       if self.size_type.get() == "pixel":
           self.pixel_size_frame.grid()
           self.doc_size_frame.grid_remove()
       else:
           self.pixel_size_frame.grid_remove()
           self.doc_size_frame.grid()
   
   # Toggle angle input visibility        
   def toggle_angle(self):
       if self.fill_type.get() == "zigzag":
           self.angle_frame.grid(row=2, column=0, columnspan=2)
       else:
           self.angle_frame.grid_remove()
   
   # Handle conversion process        
   def convert(self):
       try:
           # Validate input file
           input_file = self.file_path.get()
           if not input_file:
               tk.messagebox.showerror("Error", "Please select an input file")
               return
           
           # Get units and stroke width    
           units = self.units.get()
           stroke_width = float(self.stroke_width.get())
           
           # Get pixel size based on input method
           if self.size_type.get() == "pixel":
               pixel_size = float(self.pixel_size.get())
           else:
               doc_width = float(self.doc_width.get())
               doc_height = float(self.doc_height.get())
           
           # Convert units if needed
           if units == "in":
               stroke_width = inches_to_mm(stroke_width)
               if self.size_type.get() == "pixel":
                   pixel_size = inches_to_mm(pixel_size)
               else:
                   doc_width = inches_to_mm(doc_width)
                   doc_height = inches_to_mm(doc_height)
           
           # Calculate pixel size from document dimensions
           if self.size_type.get() == "document":
               img = Image.open(input_file)
               width, height = img.size
               pixel_size = get_pixel_size_from_doc_size(doc_width, doc_height, width, height)
           
           # Get angle for zigzag pattern
           angle = None
           if self.fill_type.get() == "zigzag":
               angle = float(self.angle.get())
           
           # Generate SVG    
           output_file = input_file.rsplit('.', 1)[0] + '.svg'
           bitmap_to_svg(input_file, output_file, pixel_size, stroke_width, angle)
           tk.messagebox.showinfo("Success", f"SVG saved as {output_file}")
           
       except Exception as e:
           tk.messagebox.showerror("Error", str(e))
   
   # Start GUI        
   def run(self):
       self.root.mainloop()

if __name__ == "__main__":
   app = BitmapConverterGUI()
   app.run()