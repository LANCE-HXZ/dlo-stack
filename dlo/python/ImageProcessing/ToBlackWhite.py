# 图片二值化
from PIL import Image
import numpy as np
from torchvision import transforms
import os

# 自定义灰度界限，大于这个值为黑色，小于这个值为白色
threshold = 128
table = []
for i in range(256):
    if i < threshold:
        table.append(1)  # 白底黑线
    else:
        table.append(0)
# path = '/home/lance/Data/GradientMap/'
path = '/home/lance/Data/t'
save_path = '/home/lance/Data/WhiteLine/'
# save_path = '/home/lance/Data/M_GradientMap/'
image_list = os.listdir(path)
length = len(image_list)
for k in range(length):
    img = Image.open(os.path.join(path, image_list[k]))
    img = img.resize((640, 320))
    img_ = np.array(img)
    rows, cols, h = img_.shape
    img = transforms.ToTensor()(img)
    # print(img)
    # 红底
    for i in range(rows):
        for j in range(cols):
            if img[1][i][j] > 0 or img[2][i][j] > 0:
                img[1][i][j] = 0
            else:
                img[1][i][j] = 1

    img = transforms.ToPILImage()(img)
    # img.save('predict1.bmp')
    # 模式L”为灰色图像，它的每个像素用8个bit表示，0表示黑，255表示白，其他数字表示不同的灰度。
    Img = img.convert('L')
    # Img.save("label/test1.jpg")
    # 图片二值化
    photo = Img.point(table, '1')
    photo.save(os.path.join(save_path, image_list[k][:-3] + 'png'))
    # photo.save('/home/lance/Workspaces/hxz_ws/src/dlo/python/ImageProcessing/test/testbw.png')
    print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )

    # ## 白底
    # for i in range(rows):
    #     for j in range(cols):
    #         if img[0][i][j] == 1 and img[1][i][j] == 1 and img[2][i][j] == 1:
    #             img[1][i][j] = 1
    #         else:
    #             img[1][i][j] = 0

    # img = transforms.ToPILImage()(img)
    # # img.save('predict1.bmp')
    # # 模式L”为灰色图像，它的每个像素用8个bit表示，0表示黑，255表示白，其他数字表示不同的灰度。
    # Img = img.convert('L')
    # # Img.save("label/test1.jpg")
    # # 图片二值化
    # photo = Img.point(table, '1')
    # photo.save(os.path.join(save_path, image_list[k][:-3] + 'png'))
    # # photo.save('/home/lance/Workspaces/hxz_ws/src/dlo/python/ImageProcessing/test/testbw.png')
    # print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )


# # 该方法生成的图片修改了尺寸
# def get_color_channels(img_in):
#     img_in = img_in.copy()
#     channels_num = len(img_in.shape)
#     result = []
#
#     channels = np.split(img_in, channels_num, axis=2)
#     for j in channels:
#         result.append(j.sum(axis=2))
#     return result
#
#
# img = Image.open('test.bmp')
# img = np.array(img)
# rows, cols, h = img.shape
# print(rows, cols)
# R, G, B, = get_color_channels(img)
# print(R)
#
# pic = np.ones(shape=(320, 640), dtype=np.uint8)
# print(pic)
# for i in range(rows):
#     for j in range(cols):
#         if R[i][j] == 255:
#             pic[i, j] = 0
#         else:
#             pic[i, j] = 1
#
# print(pic)
# plt.axis('off')
# plt.imshow(pic, 'gray')
# plt.savefig('test3.jpg')