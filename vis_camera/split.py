import os

trainPath = "/home/krri/HDD/Junha_file/MDE-Object-Detection-Fusion/NeWCRFs/dataset/train"
testPath = "/home/krri/HDD/Junha_file/MDE-Object-Detection-Fusion/NeWCRFs/dataset/test"
valPath = "/home/krri/HDD/Junha_file/MDE-Object-Detection-Fusion/NeWCRFs/dataset/val"

def getSplit(path,mode):
    d_path = os.path.join(path,"depth")
    c_path = os.path.join(path,"rgb")
    
    d_list = os.listdir(d_path)
    c_list = os.listdir(c_path)
    f = open(f"/home/krri/HDD/Junha_file/MDE-Object-Detection-Fusion/NeWCRFs/data_splits/{mode}.txt","w")
        
    for i,j in zip(c_list,d_list):
        f.write(f"{i} {j}\n")
    f.close

getSplit(trainPath,"train")
getSplit(testPath,"test")
getSplit(valPath,"val")