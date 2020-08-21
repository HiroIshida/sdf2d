while getopts 'g' c
do 
    case $c in
        g) FLG_DEBUG="TRUE"
    esac
done

cd build
cmake ..
make -j4

if [ "$FLG_DEBUG" = "TRUE" ]; then
    gdb out -command ../debug.gdb
else
    ./out
fi
