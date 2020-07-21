BUILD="./scripts/bundle.py build  --platform=x86_64-darwin --version=1.2.169 --engine-artifacts=dynamo-home"
BUNDLE="./scripts/bundle.py bundle  --platform=x86_64-darwin --version=1.2.169 --engine-artifacts=dynamo-home"
OPEN_FOLDER="open ./editor/release"

echo "Start bundling Defold Editor.."
(cd "./editor";eval $BUILD)
(cd "./editor";eval $BUNDLE)
eval $OPEN_FOLDER

echo "finished bundling Defold, enjoy ;)"
echo "_____________________________________________________"
