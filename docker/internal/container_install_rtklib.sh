export DEBIAN_FRONTEND=noninteractive

cd / && git clone -b rtklib_2.4.3 https://github.com/tomojitakasu/RTKLIB.git
cd /RTKLIB/lib/iers/gcc/
make
cd /RTKLIB/app/consapp
make
sudo make install
cd / && rm -rf /RTKLIB