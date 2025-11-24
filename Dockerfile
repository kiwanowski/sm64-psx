from ubuntu:18.04 as build

run apt update && apt install -y \
	binutils-mips-linux-gnu \
	gcc-mips-linux-gnu \
	build-essential \
	git \
	pkgconf \
	python3 \
	meson
run mkdir /sm64
workdir /sm64
env PATH="/sm64/tools:${PATH}"
copy --link . .
run make

#cmd echo 'usage: docker run --rm --mount type=bind,source="$(pwd)",destination=/sm64 sm64 make VERSION=us\n' \
#	'see https://github.com/n64decomp/sm64/blob/master/README.md for advanced usage'
