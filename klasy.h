#define _CRT_SECURE_NO_WARNINGS
#pragma once
#include <vector>
#include <queue>
#include <random>
#include <iostream>
#include <algorithm>

class ARXModel {
private:
    std::vector<double> A;       // wspolczynnik wyjscia
    std::vector<double> B;       // wspolczynnik wejscia
    std::queue<double> buforWejscia;
    std::queue<double> buforWyjscia;
    int opoznienie;
    double szum;
    std::default_random_engine generator;
    std::normal_distribution<double> dystrybucja;   // losowe wartosci zaklocenia

public:
    ARXModel(const std::vector<double>& a, const std::vector<double>& b, int k, double odchylenieSzumu = 0.01)
        : A(a), B(b), opoznienie(k), szum(odchylenieSzumu), dystrybucja(0.0, odchylenieSzumu) {
        // bufory dla wspó³czynników
        buforWejscia = std::queue<double>(std::deque<double>(B.size(), 0.0));
        buforWyjscia = std::queue<double>(std::deque<double>(A.size(), 0.0));
    }

    // krok w symulacji
    double krok(double input) {
        buforWejscia.push(input);
        if (buforWejscia.size() > B.size()) {
            buforWejscia.pop();
        }

        // wartosc wyjsciowa
        double wyjscie = 0.0;

        int i = 0;
        for (const auto& b : B) {
            wyjscie += b * buforWejscia.front();
            buforWejscia.pop();
            buforWejscia.push(wyjscie); 
        }

        i = 0;
        for (const auto& a : A) {
            wyjscie -= a * buforWyjscia.front();
            buforWyjscia.pop();
            buforWyjscia.push(wyjscie); 
        }

        // zaklocenie
        wyjscie += dystrybucja(generator);

        // bufor wyjsciowy
        buforWyjscia.push(wyjscie);
        if (buforWyjscia.size() > A.size()) {
            buforWyjscia.pop();
        }

        return wyjscie;
    }
};

template <typename T>
T filtr(T wartosc, T nizsza, T wyzsza) {
    return std::max(nizsza, std::min(wartosc, wyzsza));
}

class PIDController {
private:
    double kp, ki, kd;                  // wspolczynniki PID
    double skumulowanaWartoscCalki;     // skumulowana wartosc calki
    double bladPoprzedzajacy;           // poprzedzajacy blad dla cz³onu ró¿niczkuj¹cego
    double dolnyLimit, gornyLimit;      // limity przeciwnasyceniowe
    bool flagaPrzeciwnasyceniowa;       // flaga wlaczajaca przeciwnasycenie

public:
    // Konstruktor ustawiaj¹cy poczatkowe wartosci wspolczynnikow PID i ograniczen wyjscia
    PIDController(double kp, double ki, double kd, double dolnyLimit = -1.0, double gornyLimit = 1.0)
        : kp(kp), ki(ki), kd(kd), dolnyLimit(dolnyLimit), gornyLimit(gornyLimit), skumulowanaWartoscCalki(0.0), bladPoprzedzajacy(0.0),
        flagaPrzeciwnasyceniowa(true) {}

    // limity przeciwnasyceniowe
    void ustawLimity(double nizszy, double wyzszy) {
        dolnyLimit = nizszy;
        gornyLimit = wyzszy;
    }

    // resetowanie czlonow calkujacego i rozniczkujacego
    void reset() {
        skumulowanaWartoscCalki = 0.0;
        bladPoprzedzajacy = 0.0;
    }

    // obliczenie wartosc regulacji na podstawie uchybu
    double oblicz(double wartoscZadana, double wartoscProcesu) {
        double blad = wartoscZadana - wartoscProcesu;
        skumulowanaWartoscCalki += blad;
        double pochodna = blad - bladPoprzedzajacy;
        bladPoprzedzajacy = blad;

        double wyjscie = kp * blad + ki * skumulowanaWartoscCalki + kd * pochodna;

        // przeciwnasyceniowy filtr
        if (flagaPrzeciwnasyceniowa) {
            wyjscie = filtr(wyjscie, dolnyLimit, gornyLimit);
        }

        return wyjscie;
    }
};


