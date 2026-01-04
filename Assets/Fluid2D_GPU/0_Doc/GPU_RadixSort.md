# GPU Radix Sort
### GPU radix sort V1 
- (1 bit version) in one pass
    1. get input
    2. get "bit" as b
    3. "inverse 'bit'" as E (if it is 0, count as 1)(bucket of 0!!!)
    4. Scan the 1s of E as F
    5. TotalFasle = e[last] + f[last] (total number of 0)
    6. T = i - F +  TotalFasle
    7. (address) d = b ? T:F
    8. Scatter by d

- Scan process
    1. UP Scan 
    2. DOWN Scan

### GPU radix sort V2
- (n bit version) in one pass (n = 4,8...)
- total for 32/n passs if is uint
    1. get input single bit
    3. For i = 0 to 2^n -1
    4.      For j = 0 to n-1
    5.          bit[j] == i ? b[i][j] = 1 : b[i][j] = 0;
    6.      For j = 0 to n-1
    7.          F[i][j] = b[i][j].EScan();
    8. For i = 0 to 2^n -1
    4.      For j = 0 to n-1 : if b[i][j] == 1 
    5.          Offset[j] = F[i][j] + (i-1<0 ? 0 : F[i-1][n-1].IScan());
    6. bit.Scatter(Offset[]);