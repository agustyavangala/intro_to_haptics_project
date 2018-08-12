# short_project
Short project for Hanson and Mingyang

## FAQs

### Q: Error: git command not found
A: You have to install xcodetools. From terminal, run
```
xcode-select --install
```
### Q: Error: cmake command not found
A: 
1. First install homebrew. From terminal, run
```
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

2. Then install cmake using homebrew. Run
```
brew install cmake
```

### Q: Error: Eigen not found
A: Install eigen using homebrew. Run

```
brew install eigen
```

### Q: How do I pull the latest code?
A: 
1. From the terminal, first navigate to the project folder: short_project/
2. Then, do
```
git commit -am "Save changes"
git pull --rebase
```
