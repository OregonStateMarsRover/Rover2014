import itertools

def main():
    publisher_data = ["".join(x) for _, x in itertools.groupby("dfsd98sd8f68as7df56", key=str.isdigit)]
    print(publisher_data[1])
main()
