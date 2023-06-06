def substituir_coluna(nome_arquivo):
    with open('coluna_baromet4.txt', 'r') as coluna3:
        dados_coluna3 = coluna3.readlines()

    with open('data_SIL.txt', 'r') as data_sil:
        linhas_data_sil = data_sil.readlines()

    for i, linha in enumerate(linhas_data_sil):
        colunas = linha.strip().split(' ')

        if len(colunas) >= 4:
            colunas[3] = dados_coluna3[i].strip()

        linhas_data_sil[i] = ' '.join(colunas) + '\n'

    with open('data_SIL.txt', 'w') as data_sil:
        data_sil.writelines(linhas_data_sil)


# Exemplo de uso
substituir_coluna('dados.txt')
