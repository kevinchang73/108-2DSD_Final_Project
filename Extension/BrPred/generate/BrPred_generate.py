def little_endian_conv(str):
    str = str.replace("_","")
    substr_0 = "{:x}".format(int(str[0:8]  , base=2)).zfill(2).upper()
    substr_1 = "{:x}".format(int(str[8:16] , base=2)).zfill(2).upper()
    substr_2 = "{:x}".format(int(str[16:24], base=2)).zfill(2).upper()
    substr_3 = "{:x}".format(int(str[24:32], base=2)).zfill(2).upper()
    out_str = substr_3+"_"+substr_2+"_"+substr_1+"_"+substr_0
    return out_str

# modify # of the pattern here
nb_notBr   = 10
nb_interBr = 20
nb_Br      = 30

assert(nb_notBr > 0)
assert(nb_interBr > 0)
assert(nb_Br > 0)

total      = nb_notBr+nb_interBr+nb_Br

I_mem_BrPred_file = open("I_mem_BrPred","w")
with open("I_mem_BrPred_ref", "r") as f:
    for line in f:
        if "//0x04//" in line:
            front = line.find("000000000010_00000_000_01000_0010011")
            back  = front + len("000000000010_00000_000_01000_0010011")
            line = line.replace("a = 2"           , "a = {:d}".format(nb_notBr))
            line = line.replace("000000000010", "{:b}".format(nb_notBr).zfill(12))
            line = line.replace("0x002", "0x"+"{:x}".format(nb_notBr).zfill(3).upper())
            line = line.replace("13_04_20_00", little_endian_conv(line[front:back]))
        if "//0x08//" in line:
            front = line.find("000000000010_00000_000_01001_0010011")
            back  = front + len("000000000010_00000_000_01001_0010011")
            line = line.replace("b = 2"           , "b = {:d}".format(nb_interBr))
            line = line.replace("000000000010", "{:b}".format(nb_interBr).zfill(12))
            line = line.replace("0x002", "0x"+"{:x}".format(nb_interBr).zfill(3).upper())
            line = line.replace("93_04_20_00", little_endian_conv(line[front:back]))
        if "//0x0C//" in line:
            front = line.find("000000000010_00000_000_01010_0010011")
            back  = front + len("000000000010_00000_000_01010_0010011")
            line = line.replace("c = 2"           , "c = {:d}".format(nb_Br))
            line = line.replace("000000000010", "{:b}".format(nb_Br).zfill(12))
            line = line.replace("0x002", "0x"+"{:x}".format(nb_Br).zfill(3).upper())
            line = line.replace("13_05_20_00", little_endian_conv(line[front:back]))
        if "//0x58//" in line:
            line = line.replace("a+b+c = 6"       , "a+b+c = {:d}".format(total))
        I_mem_BrPred_file.write(line)        
I_mem_BrPred_file.close()

TestBed_BrPred_file = open("TestBed_BrPred.v","w")
with open("TestBed_BrPred_ref.v", "r") as f:
    for line in f:
        if "`define answer" in line:
            line = line.replace("6", "{:d}".format(total)) 
        TestBed_BrPred_file.write(line)
TestBed_BrPred_file.close()
