#!/usr/local/bin/python
import OpenEXR
import Imath, numpy, math
import scipy.misc
import struct
import sys

pi = math.pi
pt = Imath.PixelType(Imath.PixelType.FLOAT)


def ply_header(file, num_vertices):
    file.write("ply\n")
    # file.write("format ascii 1.0\n")
    file.write("format binary_little_endian 1.0\n")
    file.write("element vertex %s\n" % num_vertices)
    file.write("property float x\n")
    file.write("property float y\n")
    file.write("property float z\n")
    # file.write("property float nx\n")
    # file.write("property float ny\n")
    # file.write("property float nz\n")d
    # file.write("property float intensity\n")
    file.write("property uchar red\n")
    file.write("property uchar green\n")
    file.write("property uchar blue\n")
    file.write("end_header\n")

def write_point(file, point, color):
    output = "%s %s %s %s %s %s\n" % (point["x"], point["y"], point["z"],
                                                  # normal["x"], normal["y"], normal["z"],
                                                  color["r"], color["g"], color["b"])
    print output
    file.write(output)


def theta(pt_x, total_width):
    # azimuth / longitude
    # theta_val = float(float(pt_x * 2 * pi) / (total_width - 1)) - pi
    theta_val = float(float(pt_x * 2 * pi) / total_width)
    return theta_val

def phi(pt_y, total_height):
    # polar / latitude
    # phi_val = (pi/2) - (float(float(pt_y * pi) / (total_height - 1)))
    phi_val = float(float(pt_y * pi) / total_height)
    return phi_val

def radial(z, phi):
    radial_val = float(z * float(math.cos(phi)))
    return radial_val

def cart_coord(t, p, rho):
    x = float(rho * math.cos(t) * math.sin(p))
    y = float(rho * math.sin(t) * math.sin(p))
    z = float(rho * math.cos(p))
    return [x, y, z]

def get_image(image):
    latlong = OpenEXR.InputFile(image)
    dw = latlong.header()['dataWindow']

    width = dw.max.x - dw.min.x + 1
    height = dw.max.y - dw.min.y + 1

    (r_str, g_str, b_str) = latlong.channels("RGB", pt)
    red = numpy.fromstring(r_str, dtype=numpy.float32)
    red.shape = (height, width)  # (row, col)
    green = numpy.fromstring(g_str, dtype=numpy.float32)
    green.shape = (height, width)  # (row, col)
    blue = numpy.fromstring(b_str, dtype=numpy.float32)
    blue.shape = (height, width)  # (row, col)

    return red, green, blue, width, height

def get_rho(depthmap):
    depthmap = OpenEXR.InputFile(depthmap)
    dw = depthmap.header()['dataWindow']

    width = dw.max.x - dw.min.x + 1
    height = dw.max.y - dw.min.y + 1

    z_str = depthmap.channel('R', pt)
    z = numpy.fromstring(z_str, dtype=numpy.float32)
    z.shape = (height, width)  # (row, col)
    return z

def convert_to_sRGB(color):
    cc = None
    if color <= 0.0031308:
        cc = color * 12.92
    else:
        cc = 1.055 * pow(color, 1.0 / 2.4) - 0.055;
    return cc


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: jpointcloud.py <latlong.exr> <depthmap.exr> <output.ply>")
        sys.exit(1)
    latlong_file = sys.argv[1]
    depthmap_file = sys.argv[2]
    output_file = sys.argv[3]
    


    (red, green, blue, width, height) = get_image(latlong_file)
    rho_vals = get_rho(depthmap_file)


    # open and write as ascii
    f = open(output_file, "w+")
    ply_header(f, width * height)
    f.close()

    # open and write as binary
    f = open(output_file, "ab")
    for col in range(0, width):
        for row in range(0, height):
            p_rad = phi(row, height)
            t_rad = theta(col, width)
            rho = rho_vals[row, col]
            point = cart_coord(t_rad, p_rad, rho)
            r = int(convert_to_sRGB(red[row, col]) * 255)
            g = int(convert_to_sRGB(green[row, col]) * 255)
            b = int(convert_to_sRGB(blue[row, col]) * 255)
            intensity = 0.2126*r + 0.7152*g + 0.0722*b
            color = [r, g, b]
            # pack and write bytes
            print "writing: ",
            print "%s %s " % (point, color)
            f.write(struct.pack("<%df" % len(point), *point))
            f.write(struct.pack("<%dB" % len(color), *color))

            # write_point(f, point, color)

    f.close()
