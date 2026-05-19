#!/bin/bash

$(shell date +%Y%m%d_%H%M%S)
app_ver=${APP_VERSION:-"dev_$(date +%Y%m%d_%H%M%S)"}


if [ ! -v TARGET_SDKS ]; then
    declare -a target_platforms=(
            "ZCE_EM7231T"
            "ZCE_EM7231N"
            "OpenXR809"
            "OpenBL602"
            "OpenW800"
            "OpenW600"
            "OpenLN882H"
        )
else
    target_platforms=(${TARGET_SDKS//,/ })
fi
echo "****************************************"
echo "****************************************"
echo ""
echo "Building ZCE EM for the following platforms: ${target_platforms[*]}"
echo "Building ZCE EM with version \"${app_ver}\""
echo ""

cd /ZCE_EM_App
# silence any git ownership warnings beause this is being
# done through a docker. Better solution would be to
# pass through the git config of the host system
git config --global --add safe.directory /ZCE_EM_App

# build the targeted SDKs
for sdk in ${target_platforms[@]}; do
    echo ""
    echo "****************************************"
    echo "Building platform: ${sdk}"
    echo "****************************************"
    echo ""
    # need to make clean before each build otherwise the one SDKs
    # build process will be confused by the prior SDKs build of the
    # shared app code.
    make clean
    make APP_VERSION=${app_ver} APP_NAME=${sdk} ${sdk}
done

echo "****************************************"
echo ""
echo "Done building ZCE EM for the following platforms: ${target_platforms[*]}"
echo "Find outputs in: ../output/${app_ver}"
echo ""
