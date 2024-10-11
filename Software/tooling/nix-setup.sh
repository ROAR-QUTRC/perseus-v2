echo "Setting up git submodule repos"

SCRIPT_DIR="$(dirname "$(readlink -f $0)")"
# cd to project root so git will work properly
cd $SCRIPT_PATH/../..

# clone submodules if not already done so workspace can actually build
git submodule update --init --recursive

if ! command -v nix &>/dev/null
then
  echo "Setting up Nix"
  # install nix! https://github.com/DeterminateSystems/nix-installer
  # using the Determinate Systems installer rather than the "official" installer for its uninstall options and better error handling
  curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
else
  echo "Nix already present on system!"
fi

# add current user to nix trusted-users so we can do things like configure binary caching
if grep -Fq "$USER" /etc/nix/nix.conf
then
    echo "Nix trusted-users already set up!"
else
    echo "Adding current user to nix trusted-users..."
    echo "trusted-users = root $USER" | sudo tee -a /etc/nix/nix.conf

    # restart Nix daemon so above changes take effect
    echo "Restarting nix daemon"
    sudo systemctl restart nix-daemon
fi

# add direnv hooks to bash and zsh if not already present
if grep -Fq 'direnv hook bash' ~/.bashrc &>/dev/null
then
    echo "bash direnv hook already set up!"
elif command -v bash &>/dev/null
then
    echo "Setting up bash direnv hook..."
    echo "eval \"\$(direnv hook bash)\"" >> ~/.bashrc
else
    echo "bash not detected on system, not setting up .bashrc"
fi

if grep -Fq 'direnv hook zsh' ~/.zshrc &>/dev/null
then
    echo "zsh direnv hook already set up!"
elif command -v zsh &>/dev/null
then
    echo "Setting up zsh direnv hook..."
    echo "eval \"\$(direnv hook zsh)\"" >> ~/.zshrc
else
    echo "zsh not detected on system, not setting up .zshrc"
fi

if grep -Fq 'hide_env_diff' ~/.config/direnv/direnv.toml &>/dev/null
then
    echo "direnv hide_env_diff already set up!"
else
    mkdir -p ~/.config/direnv
    touch ~/.config/direnv/direnv.toml
    echo "[global]" >> ~/.config/direnv/direnv.toml
    echo "hide_env_diff = true" >> ~/.config/direnv/direnv.toml
fi

# cd to Software/ folder for direnv config
cd $SCRIPT_PATH/..
# allow direnv to configure based on the .envrc file in the current directory
direnv allow

echo "Done!"

