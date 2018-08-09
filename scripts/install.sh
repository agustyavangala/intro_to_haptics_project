dirs -c &&
mkdir -p stanford_project &&
pushd stanford_project &&

# install chai3d
if [ ! -d "chai3d" ]; then
	git clone git@github.com:manips-sai-org/chai3d.git
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
	unzip sai2-simulation.zip 
fi
pushd sai2-simulation-master &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release && make &&
popd && popd &&

# install sai2-common
if [ ! -d "sai2-common" ]; then
	git clone git@github.com:manips-sai-org/sai2-common.git
fi
pushd sai2-common &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release && make &&
popd && popd &&

# install project code
if [ ! -d "short_project" ]; then
	git clone git@github.com:manips-sai-org/short_project.git
fi
pushd short_project &&
sh build.sh
