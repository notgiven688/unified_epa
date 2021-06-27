ffmpeg -framerate 60 -i ./video/%05d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" output.mp4
