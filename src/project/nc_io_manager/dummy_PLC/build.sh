#!/bin/bash

# Dummy PLC Server 빌드 및 실행 스크립트

set -e  # 에러 발생 시 스크립트 중단

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 로그 함수들
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 사용법 출력
print_usage() {
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  build       - Build the project"
    echo "  clean       - Clean build directory"
    echo "  server      - Run dummy PLC server"
    echo "  test        - Run test client (automatic mode)"
    echo "  manual      - Run test client (manual mode)"
    echo "  help        - Show this help"
    echo ""
    echo "Options:"
    echo "  --port <port>    Server port (default: 5000)"
    echo "  --ip <address>   Server IP for client (default: 127.0.0.1)"
    echo "  --debug          Build in debug mode"
    echo ""
    echo "Examples:"
    echo "  $0 build --debug"
    echo "  $0 server --port 5000"
    echo "  $0 test --ip 192.168.1.22 --port 5000"
    echo "  $0 manual"
}

# 의존성 확인
check_dependencies() {
    log_info "Checking dependencies..."
    
    # CMake 확인
    if ! command -v cmake &> /dev/null; then
        log_error "CMake is not installed"
        exit 1
    fi
    
    # C++ 컴파일러 확인
    if ! command -v g++ &> /dev/null && ! command -v clang++ &> /dev/null; then
        log_error "C++ compiler (g++ or clang++) is not installed"
        exit 1
    fi
    
    # POCO 라이브러리 확인
    if ! pkg-config --exists libpoco-dev 2>/dev/null; then
        log_warning "POCO library may not be installed"
        log_info "On Ubuntu/Debian: sudo apt-get install libpoco-dev"
        log_info "On CentOS/RHEL: sudo yum install poco-devel"
        log_info "On macOS: brew install poco"
    fi
    
    log_success "Dependencies check completed"
}

# 프로젝트 빌드
build_project() {
    local build_type="Release"
    
    # 디버그 모드 확인
    for arg in "$@"; do
        if [[ "$arg" == "--debug" ]]; then
            build_type="Debug"
            break
        fi
    done
    
    log_info "Building project in $build_type mode..."
    
    # 빌드 디렉토리 생성
    mkdir -p build
    cd build
    
    # CMake 설정
    cmake -DCMAKE_BUILD_TYPE=$build_type ..
    
    # 빌드 실행
    make -j$(nproc 2>/dev/null || echo 4)
    
    cd ..
    log_success "Build completed successfully"
}

# 빌드 정리
clean_build() {
    log_info "Cleaning build directory..."
    
    if [[ -d "build" ]]; then
        rm -rf build
        log_success "Build directory cleaned"
    else
        log_warning "Build directory does not exist"
    fi
}

# 서버 실행
run_server() {
    local port=5000
    
    # 포트 옵션 파싱
    while [[ $# -gt 0 ]]; do
        case $1 in
            --port)
                port="$2"
                shift 2
                ;;
            *)
                shift
                ;;
        esac
    done
    
    if [[ ! -f "build/dummy_plc_server" ]]; then
        log_error "Server executable not found. Run '$0 build' first."
        exit 1
    fi
    
    log_info "Starting dummy PLC server on port $port..."
    log_info "Press Ctrl+C to stop the server"
    echo ""
    
    ./build/dummy_plc_server $port
}

# 테스트 클라이언트 실행 (자동 모드)
run_test_client() {
    local ip="127.0.0.1"
    local port=5000
    
    # 옵션 파싱
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ip)
                ip="$2"
                shift 2
                ;;
            --port)
                port="$2"
                shift 2
                ;;
            *)
                shift
                ;;
        esac
    done
    
    if [[ ! -f "build/test_client" ]]; then
        log_error "Test client executable not found. Run '$0 build' first."
        exit 1
    fi
    
    log_info "Running test client (automatic mode)..."
    log_info "Target: $ip:$port"
    log_info "Press Ctrl+C to stop the test"
    echo ""
    
    ./build/test_client --ip $ip --port $port --test
}

# 수동 테스트 클라이언트 실행
run_manual_client() {
    local ip="127.0.0.1"
    local port=5000
    
    # 옵션 파싱
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ip)
                ip="$2"
                shift 2
                ;;
            --port)
                port="$2"
                shift 2
                ;;
            *)
                shift
                ;;
        esac
    done
    
    if [[ ! -f "build/test_client" ]]; then
        log_error "Test client executable not found. Run '$0 build' first."
        exit 1
    fi
    
    log_info "Running test client (manual mode)..."
    log_info "Target: $ip:$port"
    echo ""
    
    ./build/test_client --ip $ip --port $port --manual
}

# 메인 로직
main() {
    if [[ $# -eq 0 ]]; then
        print_usage
        exit 1
    fi
    
    local command=$1
    shift
    
    case $command in
        build)
            check_dependencies
            build_project "$@"
            ;;
        clean)
            clean_build
            ;;
        server)
            run_server "$@"
            ;;
        test)
            run_test_client "$@"
            ;;
        manual)
            run_manual_client "$@"
            ;;
        help|--help|-h)
            print_usage
            ;;
        *)
            log_error "Unknown command: $command"
            print_usage
            exit 1
            ;;
    esac
}

# 스크립트 실행
main "$@"