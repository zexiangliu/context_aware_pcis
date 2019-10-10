#ffmpeg -r 8 -f image2 -i ./test1/frame%d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p test.mp4

ffmpeg -r 8 -i ./test2/frame%d.png -vcodec libx264 -y -an video.mp4 -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2"

