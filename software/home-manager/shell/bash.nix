{ ... }:
{
  programs.bash = {
    enable = true;
    historyControl = [ "ignoreboth" ];
    historyIgnore = [ "exit" ];
    # note: this is a modified section from /etc/skel/.bashrc
    # to set up a coloured prompt
    initExtra =
      let
        # it's impossible to escape the sequence "'${" inside ''-strings,
        # so just string-substituting the dollar symbol avoids the issue
        dollar = "$";
      in
      ''
        # set a fancy prompt (non-color, unless we know we "want" color)
        case "$TERM" in
            tmux*|xterm-color|*-256color) color_prompt=yes;;
        esac

        if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
            # We have color support; assume it's compliant with Ecma-48
            # (ISO/IEC-6429). (Lack of such support is extremely rare, and such
            # a case would tend to support setf rather than setaf.)
            color_prompt=yes
        else
            color_prompt=
        fi

        if [ "$color_prompt" = yes ]; then
            PS1='${dollar}{debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
        else
            PS1='${dollar}{debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
        fi
        unset color_prompt force_color_prompt

        # If this is an xterm set the title to user@host:dir
        case "$TERM" in
        xterm*|rxvt*)
            PS1="\[\e]0;''${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
            ;;
        *)
            ;;
        esac
      '';
  };
}
