#include <vector>
#include <cstdio>




// 濕潤土壤的典型反射率
typedef struct {
    float wavelength;  // 波長 nm
    float dry_soil;    // 乾燥土壤反射率
    float wet_soil;    // 濕潤土壤反射率
} soil_reflectance_t;

static const soil_reflectance_t soil_spectra[] = {
    {400, 0.10, 0.05},  // 濕潤土壤吸收更多藍光
    {500, 0.15, 0.08},
    {600, 0.20, 0.12},
    {630, 0.22, 0.15},
    {680, 0.25, 0.18},
    {700, 0.28, 0.20},
    {800, 0.35, 0.25},
    {900, 0.40, 0.30},
};



// 檢查是否真的測量到土壤
bool verify_measurement_target(const std::vector<float>& data)
{
    // 您的數據特徵：
    // f6(590nm) 異常高：11130-12356
    // 其他通道：200-500
    // 這可能是：
    
    if (data[6] > 10000.0f) {  // f6 異常高
        if (data[8] > 400.0f && data[7] > 250.0f) {
            return false;  // 不是土壤，可能有其他光源
        }
    }
    return true;
}
// 重新驗證您的 CI 計算
void verify_ci_calculation(const std::vector<float>& data)
{
    if (data.size() < 9) return;
    
    float f7 = data[7];  // 630nm
    float f8 = data[8];  // 680nm
    
    // 您的 CI 計算
    float your_ci = (f8 - f7) / (f8 + f7);
    
    // 但 f6 值很高，可能影響
    float f6 = data[6];  // 590nm
    
    printf("原始數據:\n");
    printf("  f6(590nm) = %.0f\n", f6);
    printf("  f7(630nm) = %.0f\n", f7);
    printf("  f8(680nm) = %.0f\n", f8);
    printf("計算的 CI = %.3f\n", your_ci);
    printf("f8/f7 比值 = %.2f\n", f8/f7);
    
    if (f6 > 10.0f * f7) {
        printf("⚠️  警告：f6 異常高，可能是黃光汙染\n");
    }
}
void diagnose_measurement_issue(const std::vector<float>& data)
{
    printf("=== 測量診斷 ===\n");
    
    // 1. 檢查數據範圍
    for (int i = 0; i < 10; i++) {
        printf("通道%d: %.0f", i+1, data[i]);
        if (i == 5) printf("  <- 異常高！");
        printf("\n");
    }
    
    // 2. 檢查比例
    float max_val = 0;
    int max_ch = 0;
    for (int i = 0; i < 10; i++) {
        if (data[i] > max_val) {
            max_val = data[i];
            max_ch = i;
        }
    }
    
    printf("\n最高值在通道 %d: %.0f\n", max_ch+1, max_val);
    
    // 3. 檢查 CI 計算
    float ci = (data[8] - data[7]) / (data[8] + data[7]);
    printf("計算的 CI: %.3f\n", ci);
    
    if (ci > 0.3f) {
        printf("❌ 異常：土壤 CI 不應 > 0.3\n");
        printf("可能原因：\n");
        printf("  1. 測量到植物而非土壤\n");
        printf("  2. 光源汙染（黃光）\n");
        printf("  3. 傳感器校準錯誤\n");
        printf("  4. 計算錯誤\n");
    }
}
// 檢測是否有單色光源汙染
void detect_monochromatic_light(const std::vector<float>& data)
{
    // 正常白光的通道比例
    // 黃光（590nm）不應特別突出
    
    float avg = 0;
    for (int i = 0; i < 8; i++) {
        avg += data[i];
    }
    avg /= 8.0f;
    
    float f6_ratio = data[5] / avg;  // f6 與平均值的比值
    
    printf("f6(590nm) 比值: %.1f\n", f6_ratio);
    
    if (f6_ratio > 5.0f) {
        printf("⚠️  檢測到強烈黃光成分\n");
        printf("可能來源：\n");
        printf("  - 鈉燈\n");
        printf("  - 黃色 LED\n");
        printf("  - 燭光/白熾燈\n");
    }
}
// 排除異常通道的 CI 計算
float calculate_robust_ci(const std::vector<float>& data)
{
    // 方法1：使用多個紅光通道
    float ci1 = (data[8] - data[7]) / (data[8] + data[7]);  // 680-630
    float ci2 = (data[8] - data[6]) / (data[8] + data[6]);  // 680-590
    
    // 如果 f6 異常，使用 ci1
    if (data[6] > 10.0f * data[7]) {
        return ci1;  // 忽略 f6
    }
    
    // 否則取平均
    return (ci1 + ci2) / 2.0f;
}

// 背景光減法
std::vector<float> subtract_background(const std::vector<float>& measurement, 
                                       const std::vector<float>& background)
{
    std::vector<float> corrected(measurement.size(), 0.0f);
    
    for (size_t i = 0; i < measurement.size(); i++) {
        corrected[i] = measurement[i] - background[i];
        if (corrected[i] < 0) corrected[i] = 0;
    }
    
    return corrected;
}
extern "C" void comprehensive_analysis(void )
{
    // 您的數據示例
    std::vector<float> sample = {
        0,      // 佔位
        274,    // f1
        440,    // f2
        383,    // f3
        39,     // f4
        470,    // f5
        12356,  // f6  <- 異常高！
        274,    // f7
        440,    // f8
        382,    // nir
        39,     // clear
        0.232,  // ci
        665.9,  // ppfd
        1834.5  // par
    };
    
    printf("=== 綜合分析 ===\n");
    
    // 1. 診斷
    diagnose_measurement_issue(sample);
    
    printf("\n");
    
    // 2. 檢測單色光
    detect_monochromatic_light(sample);
    
    printf("\n");
    
    // 3. 計算多種 CI
    float ci_standard = (sample[8] - sample[7]) / (sample[8] + sample[7]);
    float ci_robust = calculate_robust_ci(sample);
    
    printf("標準 CI: %.3f\n", ci_standard);
    printf("穩健 CI: %.3f\n", ci_robust);
    
    printf("\n");
    
    // 4. 推斷光源
    if (sample[6] / sample[7] > 20.0f) {
        printf("🔶 結論：存在強烈 590nm 光源汙染\n");
        printf("建議：\n");
        printf("  1. 關閉黃色光源\n");
        printf("  2. 使用濾光片\n");
        printf("  3. 重新校正 f6 通道\n");
    }
}