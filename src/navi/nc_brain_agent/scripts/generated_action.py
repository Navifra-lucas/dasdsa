import argparse
import sys
from jinja2 import Template

def read_template(file_path):
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except FileNotFoundError:
        print(f"Error: Template file '{file_path}' not found.")
        sys.exit(1)

def generate_code(params, header_template, cpp_template, file_name):
    # 템플릿 렌더링
    header_tpl = Template(header_template)
    cpp_tpl = Template(cpp_template)

    rendered_header = header_tpl.render(params)
    rendered_cpp = cpp_tpl.render(params)

    # 출력 파일 경로 설정
    header_file_path = f"{file_name}.h"
    cpp_file_path = f"{file_name}.cpp"

    # 헤더 파일 저장
    with open(header_file_path, "w") as file:
        file.write(rendered_header)

    # 구현 파일 저장
    with open(cpp_file_path, "w") as file:
        file.write(rendered_cpp)

    print(f"Header file has been generated and saved to {header_file_path}")
    print(f"Cpp file has been generated and saved to {cpp_file_path}")

def main():
    parser = argparse.ArgumentParser(description="Generate C++ class files from templates.")
    parser.add_argument('--class_name', type=str, required=True, help='The name of the class to generate.')
    parser.add_argument('--namespace', type=str, required=True, help='The namespace of the class.')
    parser.add_argument('--file_name', type=str, required=True, help='The base name for the header and cpp files.')
    parser.add_argument('--header_template', type=str, default='scripts/header_template.h.in', help='The header template file path.')
    parser.add_argument('--cpp_template', type=str, default='scripts/cpp_template.cpp.in', help='The cpp template file path.')

    args = parser.parse_args()

    if not args.class_name or not args.namespace or not args.file_name:
        parser.print_help()
        sys.exit(1)

    header_template = read_template(args.header_template)
    cpp_template = read_template(args.cpp_template)

    params = {
        "header_guard": f"NC_{args.class_name.upper()}_H",
        "namespace": args.namespace,
        "class_name": args.class_name,
        "header_file": args.file_name
    }

    generate_code(params, header_template, cpp_template, args.file_name)

if __name__ == "__main__":
    main()
