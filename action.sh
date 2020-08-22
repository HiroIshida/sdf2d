while getopts 'g' c
do 
    case $c in
        g) FLG_DEBUG="TRUE"
    esac
done

cd build

/opt/cmake-3.18.1-Linux-x86_64//bin/cmake -DPYTHON_EXECUTABLE=/usr/bin/python2.7 .. -DBUILD_PYTHON_INTERFACE=ON
make -j4

if [ "$FLG_DEBUG" = "TRUE" ]; then
    gdb out -command ../debug.gdb
else
    ./out
fi
