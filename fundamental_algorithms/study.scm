#!/usr/bin/racket
#lang racket

(define factorial_r_qiaoyx
  (lambda (x)
    (if (= x 1)
        x (* x (factorial_r_qiaoyx (- x 1))))))

(define factorial_l_qiaoyx
  (lambda (n)
    (do ((i n (- i 1)) (a 1 (* a i)))
        ((zero? i) a))))

(define gcd_r_qiaoyx
  (lambda (m n)
    (if (= m n) m
        (if (> m n)
            (gcd_r_qiaoyx (- m n) n)
            (gcd_r_qiaoyx m (- n m))))))

(define fibonacci_r_qiaoyx
  (lambda (x)
    (if (or (= x 1) (= x 0))
        x (+ (fibonacci_r_qiaoyx (- x 2)) (fibonacci_r_qiaoyx (- x 1))))))

(define fibonacci_t_qiaoyx
  (lambda (x r t)
    (if (= x 0)
        r (fibonacci_t_qiaoyx (- x 1) t (+ r t)))))

(factorial_r_qiaoyx 5)
(factorial_l_qiaoyx 5)
(gcd_r_qiaoyx 100 52)
(gcd_r_qiaoyx 52 100)
(fibonacci_r_qiaoyx 7)
(fibonacci_t_qiaoyx 8 0 1)
