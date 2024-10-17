#include <iostream>
#include <cmath>

using namespace std;

float alf, f1, f2;
const float L = 366.8, W = 267.0;
float b = 267.0, a = 67.25, c = 60.24, h = 75.0, m = 80.0, ANGLE = 0.0, xmin = -18.15, xmax = 18.15, x = 0.0;

void WheelAngle() {
    float w = (b - a) / 2;
    w += x;
    float L = sqrt(pow(b - w, 2) + pow(h, 2));
    float COSa = (b - w) / L;
    float AngleA = acos(COSa);
    float g = sqrt(L * L + a * a - 2 * a * L * COSa);
    float COSf1 = (g * g + L * L - a * a) / (2 * g * L);
    float f1 = acos(COSf1);
    float COSf2 = (c * c + g * g - m * m) / (2.0 * c * g);
    float f2 = acos(COSf2);
    ANGLE = AngleA + f1 + f2;
    ANGLE *= 57;
    ANGLE = round(ANGLE * 100) / 100;
    cout << ANGLE << "\n";
}

void Dichotomy(float currentAngle) {
    float L = xmin, R = xmax;
    while (R - L > 0.01) {
        x = (L + R) * 0.5;
        WheelAngle();
        if (ANGLE >= currentAngle) R = x;
        else L = x;
        cout << ANGLE << " " << x << "\n";
    }
}

void Ackerman(float R) {
    f1 = 57 * atan(L / float(R - 43.5 - W / 2));
    f2 = 57 * atan(L / float(R + 43.5 + W / 2));
}

int main() {
    float R;
    cin >> R;
    Ackerman(R);
    cout << 68.08 - f1 << " " << 68.08 + f2 << "\n";
    float currentAngle = 68.08 - f1;
    Dichotomy(currentAngle);
    float x1 = x;
    currentAngle = 68.08 + f2;
    Dichotomy(currentAngle);
    cout << -(x1 - x) / 2;
    return 0;
}
