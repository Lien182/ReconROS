reconos=`dirname $BASH_SOURCE`/..
reconos=`cd $reconos && pwd`
echo "RECONOS=$reconos"
echo "PYTHONPATH=$reconos/tools/_pypack"
echo "PATH=$reconos/tools:\$PATH"

export RECONOS=$reconos
export PYTHONPATH=$reconos/tools/_pypack
export PATH=$reconos/tools:$PATH