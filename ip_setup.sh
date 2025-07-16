#!/bin/bash

# === Validación de argumentos ===
if [ "$#" -ne 3 ]; then
  echo "❌ Uso: $0 <letra_de_red> <contraseña_wifi> <ultimo_octeto_ip>"
  echo "   Ejemplo: $0 C mipasswifi 15  →  IP: 192.168.13.15"
  exit 1
fi

LETTER=$(echo "$1" | tr '[:lower:]' '[:upper:]')
PASSWORD="$2"
OCTET="$3"

# Validar la letra
if [[ ! "$LETTER" =~ ^[A-D]$ ]]; then
  echo "❌ Letra de red inválida. Usa A, B, C o D."
  exit 1
fi

# Validar el último octeto (debe estar entre 2 y 254)
if ! [[ "$OCTET" =~ ^[0-9]+$ ]] || [ "$OCTET" -lt 2 ] || [ "$OCTET" -gt 254 ]; then
  echo "❌ Último octeto inválido. Debe ser un número entre 2 y 254."
  exit 1
fi

# === Configuración de red ===
SSID="FIELD_${LETTER}_5G"
CON_NAME="$SSID"
IFACE="wlan0"  # Cambia si tu interfaz es diferente
STATIC_IP="192.168.13.$OCTET/16"
GATEWAY="192.168.13.1"
DNS="8.8.8.8 1.1.1.1"

# Eliminar conexión existente si ya existe (opcional)
nmcli con delete "$CON_NAME" &>/dev/null

# Crear nueva conexión con IP estática
nmcli con add \
  type wifi \
  ifname "$IFACE" \
  con-name "$CON_NAME" \
  ssid "$SSID" \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "$PASSWORD" \
  ipv4.addresses "$STATIC_IP" \
  ipv4.gateway "$GATEWAY" \
  ipv4.dns "$DNS" \
  ipv4.method manual

# Activar la conexión
nmcli con up "$CON_NAME"

echo "✅ Conexión '$CON_NAME' configurada con IP estática: $STATIC_IP"

