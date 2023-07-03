Import("env", "projenv")
from shutil import copyfile

# Dump construction environment (for debug purpose)
# print(env.Dump())
# print(projenv.Dump())

FILENAME_BUILDNO = 'versioning'
build_no = "1"

# copy firmware.bin to location
def save_bin(*args, **kwargs):
    target = str(kwargs['target'][0])
    copyfile(target, "$PROJECT_DIR\\..\\firmware_%s.bin" % build_no)

try:
    with open(FILENAME_BUILDNO) as f:
        build_no = f.readline()
except:
    print('Starting build number from 1..')
    build_no = 1

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", save_bin)

