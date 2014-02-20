import itertools

def main():
    publisher_string = "s2f2r90f35s1"
    publisher_data = ["".join(x) for _, x in itertools.groupby(publisher_string, key=str.isdigit)]
    #Publisher Data is an array of alternating chars and integers
    print(publisher_data)
main()
