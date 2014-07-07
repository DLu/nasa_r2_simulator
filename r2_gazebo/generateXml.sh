for file in $PWD/robots/*.xacro
do
    filename=${file%.*o}
    echo $file
    rosrun xacro xacro.py $file > "$filename.xml"
    mv "$filename.xml" ./robots
done
exit 0
