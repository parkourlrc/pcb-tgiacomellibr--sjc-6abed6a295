#!/usr/bin/env zsh
# ================================================================
# board_ident_sigs.sh
# Script to run signal identification sequence on SJC board
# with automatic logging using sjc_write_log.
#
# Flow:
#   1. Configure control frequency
#   2. Flush log serial
#   3. Start sjc_write_log in background
#   4. Run identification signals
#   5. Cleanup: disable control + stop sjc_write_log
# ================================================================

cleanup() {
    echo "[CLEANUP] Disabling control..."
    (cd ../build && ./sjc_write_control_mode_ref "$CMD_PORT" "$ID" "$BAUD" 0 0) || true

    if [ -n "$LOG_PID" ]; then
        if ps -p $LOG_PID > /dev/null; then
            echo "[CLEANUP] Stopping sjc_write_log (PID=$LOG_PID)..."
            kill $LOG_PID
            wait $LOG_PID 2>/dev/null
        fi
    fi

    echo "[CLEANUP] Done."
}
trap 'cleanup; exit 130' INT
trap 'cleanup' EXIT

# ================================
# Input validation
# ================================
if [ $# -ne 3 ]; then
    echo "Usage: $0 ID FREQ AMP"
    echo "  ID   : Board address"
    echo "  FREQ : Control loop frequency [Hz]"
    echo "  AMP  : Excitation amplitude [V]"
    exit 1
fi

# ================================
# Internal configuration
# ================================
CMD_PORT="/dev/ttyUSB0"   # Porta serial para comandos
LOG_PORT="/dev/ttyACM0"   # Porta serial de log (fixa)

BAUD=921600               # baud para comandos
LOG_BAUD=4000000          # baud para log

TYPE="Home"             # >>> Configurável: Bridge / Rope / Trolley
TAKE=2                    # >>> Configurável: número da tomada (Take)

# ================================
# Script arguments
# ================================
ID="$1"                   # ID do board
FREQ="$2"                 # Frequência de controle (Hz)
AMP="$3"                  # Amplitude de excitação (V)

# ================================
# Logging configuration
# ================================
FILE_PATH="$HOME/Logs/joints"
mkdir -p "$FILE_PATH"

# Removida a hora, só data YYYY-MM-DD
FILE_NAME="${TYPE}-ol_${AMP}V_${FREQ}Hz_$(date +%F)_Take-${TAKE}.csv"
LOG_FILE="${FILE_PATH}/${FILE_NAME}"

echo "[INFO] Running signals on $CMD_PORT (ID=$ID, FREQ=$FREQ Hz, AMP=$AMP V)"
echo "[INFO] Logging from $LOG_PORT to $LOG_FILE"

# ================================
# Configure frequency first
# ================================
cd ../build || exit 1
./sjc_write_control_freq "$CMD_PORT" "$ID" "$BAUD" "$FREQ"
sleep 2

# ================================
# Flush log serial before starting logger
# ================================
if [ -e "$LOG_PORT" ]; then
    echo "[INFO] Flushing log serial port $LOG_PORT..."
    stty -F "$LOG_PORT" raw -echo
    dd if="$LOG_PORT" of=/dev/null bs=1024 count=1 2>/dev/null || true
fi

# ================================
# Launch logger
# ================================
echo "[INFO] Starting sjc_write_log..."
./sjc_write_log "$LOG_PORT" "$LOG_BAUD" "$LOG_FILE" &
LOG_PID=$!

sleep 2
if ! ps -p $LOG_PID > /dev/null; then
    echo "[ERROR] sjc_write_log failed to start"
    exit 1
fi
echo "[INFO] sjc_write_log started with PID $LOG_PID"

# ================================
# Run identification signals
# ================================

# --- MID PRBS (5 Hz) ---
echo "[INFO] MID PRBS (5 Hz)"
./sjc_write_control_mode_ref "$CMD_PORT" "$ID" "$BAUD" 7 "$AMP"
sleep 27

# --- SINE + PRBS ---
echo "[INFO] SINE + PRBS (1, 2.5, 5 Hz + PRBS 10 Hz)"
./sjc_write_control_mode_ref "$CMD_PORT" "$ID" "$BAUD" 9 "$AMP"
sleep 12

# --- PULSE FAST (4 Hz) ---
echo "[INFO] PULSE FAST (4 Hz)"
./sjc_write_control_mode_ref "$CMD_PORT" "$ID" "$BAUD" 10 "$AMP"
sleep 7

# --- PULSE SLOW (2 Hz) ---
echo "[INFO] PULSE SLOW (2 Hz)"
./sjc_write_control_mode_ref "$CMD_PORT" "$ID" "$BAUD" 11 "$AMP"
sleep 12

echo "[INFO] Sequence finished."
