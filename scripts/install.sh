mkdir -p stanford_project &&
pushd stanford_project &&

# install chai3d
git clone git@github.com:manips-sai-org/chai3d.git &&
pushd chai3d &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
popd && popd &&

# copy sai2-simulation
popd &&
mv sai2-simulation.zip stanford_project/ &&
pushd stanford_project &&

# install sai2-simulation
unzip sai2-simulation.zip &&
pushd sai2-simulation &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
popd && popd &&

# install sai2-common
git clone git@github.com:manips-sai-org/sai2-common.git &&
pushd sai2-common &&
mkdir -p build &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
popd && popd &&

# install project code
git clone git@github.com:shameekganguly/short_project.git &&
pushd short_project &&
sh build.sh
