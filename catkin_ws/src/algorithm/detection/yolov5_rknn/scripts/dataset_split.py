#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import os
import random
from shutil import copy2


def data_set_split(src_folder, target_folder, train_scale=0.7, val_scale=0.15, test_scale=0.15):
    # 检测是否存在目标文件夹，已经目标文件夹下的images和labels文件夹
    split_image_path = os.path.join(target_folder, "images")
    if os.path.isdir(split_image_path):
        pass
    else:
        os.makedirs(split_image_path)
    split_label_path = os.path.join(target_folder, "labels")
    if os.path.isdir(split_label_path):
        pass
    else:
        os.makedirs(split_label_path)

    cmd = input("若目标文件夹下存在文件，将被全部删除\n按 n/N键 取消本次操作，按其它键继续\n>>>")
    if cmd in ['n', 'N']:
        print("已保留目标文件夹下的文件，提前退出")
        exit()

    # 在目标 images 和 labels 下分别创建train  val  test
    split_names = ['train', 'val', 'test']
    for split_name in split_names:
        split_image_path = os.path.join(target_folder, "images", split_name)
        split_label_path = os.path.join(target_folder, "labels", split_name)
        if os.path.isdir(split_image_path):
            pass
        else:
            os.mkdir(split_image_path)
        if os.path.isdir(split_label_path):
            pass
        else:
            os.mkdir(split_label_path)

    # class_names = os.listdir(src_folder)  # ['images', 'labels']

    src_image_folder = src_folder + "images/"
    src_label_folder = src_folder + "labels/"

    src_image_names = os.listdir(src_image_folder)
    src_label_names = os.listdir(src_label_folder)
    if 'classes.txt' in src_label_names:
        src_label_names.remove('classes.txt')
        
    src_image_length = len(src_image_names)
    src_label_length = len(src_label_names)

    # 双向检查
    check_err = 0
    for i in range(src_image_length):
        if src_image_names[i][:-3] + 'txt' in src_label_names:
            pass
        else:
            print(src_image_names[i] + ' 没有对应标签')
            check_err = 1
    for i in range(src_label_length):
        if src_label_names[i][:-3] + 'jpg' in src_image_names:
            pass
        else:
            print(src_label_names[i] + ' 没有对应图片')
            check_err = 1
    if check_err:
        print("双向检查错误")
        exit()

    # 根据图片编号，乱序排列源图片文件
    data_index_list = list(range(src_image_length))
    random.shuffle(data_index_list)

    # 根据分类百分比记录各集的图片数量
    train_stop_flag = src_image_length * train_scale
    val_stop_flag = src_image_length * (train_scale + val_scale)
    # test_stop_flag = src_image_length * (train_scale + val_scale + test_scale)

    # 记录对应文件夹名
    image_train_folder = target_folder + "images/" + split_names[0] + "/"
    image_val_folder = target_folder + "images/" + split_names[1] + "/"
    image_test_folder = target_folder + "images/" + split_names[2] + "/"
    label_train_folder = target_folder + "labels/" + split_names[0] + "/"
    label_val_folder = target_folder + "labels/" + split_names[1] + "/"
    label_test_folder = target_folder + "labels/" + split_names[2] + "/"

    current_idx = 0
    train_num = 0
    val_num = 0
    test_num = 0

    for i in data_index_list:
        src_image_path = src_image_folder + src_image_names[i]
        src_label_path = src_label_folder + src_image_names[i][:-3] + 'txt'
        if current_idx <= train_stop_flag:
            copy2(src_image_path, image_train_folder)
            copy2(src_label_path, label_train_folder)
            print("{}复制到了{}".format(src_image_path, image_train_folder))
            print("{}复制到了{}".format(src_label_path, label_train_folder))
            train_num = train_num + 1
        elif (current_idx > train_stop_flag) and (current_idx <= val_stop_flag):
            copy2(src_image_path, image_val_folder)
            copy2(src_label_path, label_val_folder)
            print("{}复制到了{}".format(src_image_path, image_val_folder))
            print("{}复制到了{}".format(src_label_path, label_val_folder))
            val_num = val_num + 1
        else:
            copy2(src_image_path, image_test_folder)
            copy2(src_label_path, label_test_folder)
            print("{}复制到了{}".format(src_image_path, image_test_folder))
            print("{}复制到了{}".format(src_label_path, label_test_folder))
            test_num = test_num + 1

        current_idx = current_idx + 1
        print()


if __name__ == '__main__':
    src_data_folder = "/home/orangepi/dataset/original/"   # todo 修改你的原始数据集路径
    target_data_folder = "/home/orangepi/dataset/split/"  # todo 修改为你要存放的路径
    data_set_split(src_data_folder, target_data_folder,
                   train_scale=0.7, 
                   val_scale=0.15, 
                   test_scale=0.15)
