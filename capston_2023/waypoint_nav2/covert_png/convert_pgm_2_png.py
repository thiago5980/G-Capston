from PIL import Image

def convert_pgm_to_png(pgm_path, png_path):
    # 이미지 파일 열기
    with Image.open(pgm_path) as img:
        # png로 저장
        img.save(png_path)

if __name__ == "__main__":
    pgm_path = "outside_smoke.pgm"
    png_path = "outside_smoke.png"
    convert_pgm_to_png(pgm_path, png_path)
