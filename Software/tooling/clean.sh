SCRIPT_DIR="$(dirname "$(readlink -f $0)")"
# cd to project root so git will work properly
cd $SCRIPT_DIR/../..

rm -rf **/result **/build/ **/log/ **/install/