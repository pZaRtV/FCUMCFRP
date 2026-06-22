import socket

# Atur IP Laptop dan Port (0.0.0.0 berarti mendengarkan dari semua antarmuka jaringan)
UDP_IP = "0.0.0.0" 
UDP_PORT = 8888

# Membuat socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Laptop siap mendengarkan data UDP di port {UDP_PORT}...")

try:
    while True:
        # Menerima paket data (buffer size 1024 bytes)
        data, addr = sock.recvfrom(1024) 
        
        # Decode data dari byte menjadi string text
        pesan = data.decode('utf-8')
        
        print(f"Diterima dari {addr}: {pesan}")
        
except KeyboardInterrupt:
    print("\nProgram dihentikan.")
    sock.close()