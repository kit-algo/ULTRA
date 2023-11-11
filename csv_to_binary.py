import csv

def csv_to_binary(csv_filename, binary_filename):
    with open(csv_filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        data = list(csv_reader)

    with open(binary_filename, 'wb') as binary_file:
        for row in data:
            binary_file.write(bytes(','.join(row) + '\n', encoding='utf-8'))
