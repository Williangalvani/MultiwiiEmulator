__author__ = 'Will'



def itoa(number):
        extract_digits = []
        while(number>10):
            extract_digits.append(number%10)
            number /= 10;
        return extract_digits

print itoa(12345)