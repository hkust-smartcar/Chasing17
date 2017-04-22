for cnt in range(10):
    text = open("info_" + str(cnt) + ".txt", "r")
    string = text.read()
    str2 = ""
    for char in string:
        str2 += char + '\n'
    text.close()
    text = open("info_" + str(cnt) + ".txt", "wt")
    text.write(str2)
        
