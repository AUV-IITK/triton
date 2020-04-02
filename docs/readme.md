GuideLines for generating documentation

# Auto-Generate Documentation by Doxygen using the following steps

1. Generate the deconfig file by the command $ doxygen -g deconfig
2. Open the deconfig file and source the required directory. It is pre-assigned to current working directory.
3. Setup the require working output path under OUTPUT_DIRECTORY.
4. Set the parameters INLINE_SOURCES, SOURCE_BROWSER and RECURSIVE to YES
5. Run the command $ doxygen deconfig 

# Updating documentation after changes

1. Run the command $ doxygen deconfig

# For Downloading Doxygen and Setup 

http://www.doxygen.nl/download.html

# Official Documentation