#include "quicksort.h"

int partition(std::vector<Aftr::Vector> &verts, int start, int end, int pos)
{
    Aftr::Vector pivot = verts[start];

    int count = 0;
    for (int i = start + 1; i <= end; i++) {
        if (verts[i][pos] <= pivot[pos])
            count++;
    }

    // Giving pivot element its correct position
    int pivotIndex = start + count;
    Aftr::Vector temp = verts[pivotIndex];
    verts[pivotIndex] = verts[start];
    verts[start] = temp;

    // Sorting left and right parts of the pivot element
    int i = start, j = end;

    while (i < pivotIndex && j > pivotIndex) {

        while (verts[i][pos] <= pivot[pos]) {
            i++;
        }

        while (verts[j][pos] > pivot[pos]) {
            j--;
        }

        if (i < pivotIndex && j > pivotIndex) {
            Aftr::Vector temp = verts[i];
            verts[i] = verts[j];
            verts[j] = temp;
            i++; j--;
        }
    }

    return pivotIndex;
}

void quickSort(std::vector<Aftr::Vector> &verts, int start, int end, int pos) 
{
	if (start >= end)
		return;

	int p = partition(verts, start, end, pos);
	quickSort(verts, start, p - 1, pos);
	quickSort(verts, p + 1, end, pos);
}