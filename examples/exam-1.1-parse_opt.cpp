#include <iostream>
#include <cstdlib> // 用于 std::exit
#include <stdexcept> // 用于 std::invalid_argument

// 函数声明
double add(double a, double b);

double subtract(double a, double b);

double multiply(double a, double b);

double divide(double a, double b);

int main(int argc, char *argv[]) {
    // 检查参数数量
    if (argc != 4) {
        std::cerr << "Usage: calculator <num1> <operation> <num2>\n";
        std::cerr << "Operations: +, -, *, /\n";
        return 1;
    }

    // 解析输入参数
    double num1 = std::atof(argv[1]);
    std::string operation = argv[2];
    double num2 = std::atof(argv[3]);
    double result;

    try {
        if (operation == "+") {
            result = add(num1, num2);
        } else if (operation == "-") {
            result = subtract(num1, num2);
        } else if (operation == "*") {
            result = multiply(num1, num2);
        } else if (operation == "/") {
            if (num2 == 0) {
                throw std::invalid_argument("Division by zero is not allowed.");
            }
            result = divide(num1, num2);
        } else {
            std::cerr << "Invalid operation: " << operation << "\n";
            return 1;
        }

        std::cout << "Result: " << result << "\n";
    } catch (const std::invalid_argument &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

// 函数定义
double add(double a, double b) {
    return a + b;
}

double subtract(double a, double b) {
    return a - b;
}

double multiply(double a, double b) {
    return a * b;
}

double divide(double a, double b) {
    return a / b;
}