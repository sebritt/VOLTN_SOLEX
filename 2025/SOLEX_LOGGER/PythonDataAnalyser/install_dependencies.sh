#!/bin/bash

# Vérifie si pip est installé
if ! command -v pip &> /dev/null; then
    echo "pip n'est pas installé. Veuillez l'installer d'abord."
    exit 1
fi

echo "Installation des packages Python requis avec --break-system-package"

# Installe Flask
pip install flask --break-system-package

# Installe pyserial (inclut serial.tools.list_ports et serial)
pip install pyserial --break-system-package

echo "Installation terminée."
