# GitHub Repository Maintenance

Just a few things in here which go over some standards in the repo.

## Tags

All tags have a two-digit number prepended to their name, which enforces a specific ordering, as well as groups them.
This number also determines their colour.

| First Digit | Meaning                                                                           | Colour    |
| ----------- | --------------------------------------------------------------------------------- | --------- |
| 0 or 1      | Issue classification/"meta-tags"[^meta-tags] - bug, enhancement, etc.             | Red       |
| 2 or 3      | Software-related tags                                                             | Teal      |
| 4           | Task specific payload tags                                                        | Yellow    |
| 5           | Subsystem classification (if applicable) - this one's less strict than the others | Dark blue |
| 6           | Build system - Nix, CMake, etc                                                    | Purple    |
| 7           | Hardware specific issues                                                          | Green     |

[^meta-tags]: By this I mean tags which generally don't classify what the issue involves, but do change how to deal with it.
