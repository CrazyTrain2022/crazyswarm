from chooser import *
from pycrazyswarm import *

allcfs = Crazyswarm().allcfs
for cf in allcfs.crazyflies:
    print(cf.prefix)

# print(selected_cfs())