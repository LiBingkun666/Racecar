1，labelme标注，得到标注后的json
2、转化的时候需要修改： D:\Anaconda3\Lib\site-packages\labelme\cli\json_to_dataset.py 中的类别个数：如：
        label_name_to_value = {"_background_": 0,
                                   "xuxian":1,
                                   "yellow":2,
                                   "shixian":3}
2、将原始图片和json放到同一个文件夹下
3、修改 seglabels.txt中的 分割类别
4、运行 process中的 labelme2segvoc
    python labelme2segvoc.py “2、中的路径”  ./segvoc --labels ./seglabels.txt
5、 python generate_mask.py --input-dir ./segvoc/SegmentationClass --output-dir ./masklabels
6、 train_custom.py