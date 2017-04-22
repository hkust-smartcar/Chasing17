from PIL import Image

input("Press Enter to start...")

cnt = 0
for cnt in range(10):
    pxcnt = 0

    text = open("info_" + str(cnt) + ".txt", "r")
    img = Image.new("1", (128, 480))
    pixels = img.load()

    for pixel in text:
        pixel = int(pixel.rstrip())
        pixels[pxcnt % 128, pxcnt / 128] = pixel
        pxcnt += 1
        

    img.save(str(cnt) + ".bmp")
    img.show()
    input("Press Enter to continue...")
    
    
