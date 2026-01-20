**DO THE FOLLOWING TO GET ALL REQUIRED REPOS INCUDING SUBMODULSAND RECURSIVE MODULES(see setup guide for OMY):**

```
git clone https://github.com/kiashik/throw_and_catch.git
cd throw_and_catch
git submodule update --init --recursive
```

How to update a specific submodule (replace submodule name):
```
cd throw_and_catch_ws/src/image_pipeline
git pull origin jazzy    # specific branch name is jazzy
cd ../../..
git add throw_and_catch_ws/src/image_pipeline
git commit -m "Update image_pipeline submodule"
```
