# Install doxygen
sudo apt-get install texlive-latex-extra
sudo apt-get install ghostscript
sudo apt-get install doxygen

# Install graphviz
sudo apt-get install graphviz

# mavlink
sudo apt install python3 python3-pip
git clone https://github.com/mavlink/mavlink.git --recursive
python3 -m pip install -r pymavlink/requirements.txt