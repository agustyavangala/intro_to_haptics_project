dirs -c &&
mkdir -p stanford_project &&
pushd stanford_project &&

# install chai3d
if [ ! -d "chai3d" ]; then
	git clone git@github.com:manips-sai-org/chai3d.git || exit 1
fi
pushd chai3d &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release && make &&
popd && popd &&

# copy sai2-simulation
popd &&
cp sai2-simulation.zip stanford_project/ &&
pushd stanford_project &&

# install sai2-simulation
if [ ! -d "sai2-simulation" ]; then
	unzip sai2-simulation.zip  || exit 1
fi
pushd sai2-simulation-master &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release && make &&
popd && popd &&

# install tinyxml
if [ ! -d "tinyxml2" ]; then
	git clone git@github.com:leethomason/tinyxml2.git || exit 1
fi
pushd tinyxml2 && 
git checkout 4.0.1 &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release && make &&
echo "Please enter computer password: " &&
sudo make install &&
popd && popd &&

# install rbdl
# TODO: make SHA free paths
if [ ! -d "rbdl-rbdl-0879ee8c548a" ]; then
	curl https://bitbucket.org/rbdl/rbdl/get/default.zip -o rbdl.zip &&
	unzip rbdl.zip || exit 1
fi
pushd rbdl-rbdl-0879ee8c548a &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON -D && make &&
echo "Please enter computer password: " &&
sudo make install &&
popd && popd &&


# install sai2-common
if [ ! -d "sai2-common" ]; then
	git clone git@github.com:manips-sai-org/sai2-common.git  || exit 1
fi
pushd sai2-common &&
git checkout f1c63fb0e3a0382c46c1ebd04e5078a76090689f &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release && make &&
popd && popd &&

# install project code
if [ ! -d "short_project" ]; then
	git clone git@github.com:shameekganguly/short_project.git || exit 1
fi
pushd short_project &&
sh build.sh
