import csv
import sys
import serial
import struct

# Citirea variabilelor de intrare de la tastatură
curent_prag = float(input("Introduceti de la tastatura valoarea pentru curentul de prag [A]: "))
tensiune_prag = float(input("Introduceti de la tastatura valoarea pentru tensiunea de prag [V]: "))
numar_cicluri = int(input("Introduceti de la tastatura numarul de cicluri de testare dorit: "))

# Crearea fișierului CSV și scrierea headerului
csv_file = open('masuratori.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Nr_ciclu', 'Curent_incarcare', 'Tensiune_incarcare', 'Curent_descarcare', 'Tensiune_descarcare',
                     'Curent_prag', 'Tensiune_prag', 'Numar_total_cicluri'])

# Scrierea liniei de date fictive cu -1 ca prima valoare și 0 pentru celelalte valori care ar fi trebuit să fie primite prin port serial
dummy_data = [-1, 0, 0, 0, 0, curent_prag, tensiune_prag, numar_cicluri]
csv_writer.writerow(dummy_data)

try:
    # Închiderea fișierului CSV
    csv_file.close()

    # Configurarea portului serial
    ser = serial.Serial('COM1', 9600)

    # Redeschiderea fișierului CSV în modul append
    csv_file = open('masuratori.csv', 'a', newline='')
    csv_writer = csv.writer(csv_file)

    # Transmiterea variabilelor de intrare către modulul FT232HL ca bytes
    ser.write(struct.pack('f', curent_prag))
    ser.write(struct.pack('f', tensiune_prag))
    ser.write(struct.pack('i', numar_cicluri))

    # Buclea principală pentru citirea și procesarea datelor
    while True:
        # Citirea datelor de pe portul serial
        data = ser.readline().decode().strip().split()

        # Verificarea dacă numărul de valori este un multiplu de 5
        if len(data) % 5 == 0:
           # Conversia datelor în numere cu virgulă de tip float cu trei zecimale
            data = [format(float(value), '.3f') for value in data]

            # Calcularea numărului de rânduri în matrice
            num_rows = len(data) // 5

             # Scrierea fiecărui rând al matricei într-un rând separat în fișierul CSV
            for i in range(num_rows):
                start_index = i * 5
                end_index = start_index + 5
                row_data = data[start_index:end_index] + [curent_prag, tensiune_prag, numar_cicluri]
                csv_writer.writerow(row_data)

except KeyboardInterrupt:
    print("Detecție întrerupere tastatură. Se încheie...")
    sys.exit(0)

except serial.SerialException:
    print("Eroare de conexiune serială.")

finally:
    # Închiderea portului serial și a fișierului CSV
    ser.close()
    csv_file.close()
