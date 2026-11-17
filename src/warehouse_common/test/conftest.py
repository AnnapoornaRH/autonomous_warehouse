import os
import sys

# make `import warehouse_common` work when pytest is run straight from the source tree
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
