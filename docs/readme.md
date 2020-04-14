# GuideLines for generating documentation

Our `gh-pages` branch will remain protected, with changes being allowed only by the maintainers.

### Updating documentation after changes

We are using the `gh-pages` branch for maintaining our documentation. Instructions are available [here](https://github.com/m-a-d-n-e-s-s/madness/issues/104). It is slightly confusing at the first site, but keeping the documentation on master produces a lot of file changes, making it intractable.

### Auto-Generate Documentation by Doxygen (setting it up for a new repository):

1. Generate the deconfig file:  `$ doxygen -g deconfig`
2. Open the deconfig file and source the required directory. It is pre-assigned to current working directory.
3. Setup the require working output path under `OUTPUT_DIRECTORY`.
4. Set the parameters `INLINE_SOURCES`, `SOURCE_BROWSER` and `RECURSIVE to YES`
5. Run the command `$ doxygen deconfig` to generate documentation


### For Downloading Doxygen and Setup 
http://www.doxygen.nl/download.html