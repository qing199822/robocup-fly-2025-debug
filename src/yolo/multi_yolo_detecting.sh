typhoon_h480_num=6
vehicle_num=0

while(( $vehicle_num< typhoon_h480_num)) 
do
    python yolo11n.py typhoon_h480 $vehicle_num&
    let "vehicle_num++"
done
